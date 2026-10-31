#!/usr/bin/env python3
"""
Stable FastSLAM endurance node.

This version intentionally stays close to the previous working FastSLAM node:
- Same CUDA FastSLAM core path.
- Same odom + cone_array synchronisation.
- Same map publishing structure.
- Adds /laps-based map freeze.
- After freeze, the landmark section of every particle is overwritten with the
  frozen best-particle map to prevent ghost cones becoming permanent.

This is the safer integration version. It does NOT introduce a separate MCL
localiser, so there are fewer new moving parts and fewer hyperparameters.
"""

import os
import math
import traceback

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Header, UInt16
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from common_msgs.msg import ConeArray, Cone
from ament_index_python.packages import get_package_share_directory

import pycuda.driver as cuda
import pycuda.autoinit  # noqa: F401

DEBUG_GPU = False

# ------------------ FastSLAM Core Imports ------------------
try:
    # fastslam64 is vendored (slam/fastslam64, see its README): the kernel fixes live in its source.
    from slam.fastslam64.lib.particle3 import FlatParticle
    from slam.fastslam64.lib.common import CUDAMemory, rescale, get_pose_estimate, resample
    from slam.fastslam64.cuda import fastslam as fastslam_cuda
    from slam.fastslam64.lib.utils import dotify
except Exception as e:
    raise SystemExit(f"Failed to import FastSLAM core components: {e}")


KERN_FLOAT = np.float64
WEIGHT_DTYPE = np.float64

# ------------------ Config Helpers ------------------
def cfg_get(obj, name, default):
    """Return config value, but treat missing or YAML null/blank as default."""
    value = getattr(obj, name, default)
    if value is None:
        return default
    return value


def load_config(use_sim: bool):
    share_dir = get_package_share_directory("slam")
    filename = "sim_config.yaml" if use_sim else "real_config.yaml"
    config_path = os.path.join(share_dir, "config", filename)

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not data:
        raise RuntimeError(f"Config file is empty: {config_path}")

    config = dotify(data)

    # Original required fields.
    config.N = int(cfg_get(config, "N", 512))
    config.THREADS = int(cfg_get(config, "THREADS", 256))
    config.MAX_LANDMARKS = int(cfg_get(config, "MAX_LANDMARKS", 600))
    config.SEED = int(cfg_get(config, "SEED", 42))
    config.THRESHOLD = float(cfg_get(config, "THRESHOLD", 3.0))

    if config.N % config.THREADS != 0:
        raise RuntimeError(
            f"N must be divisible by THREADS. Got N={config.N}, THREADS={config.THREADS}"
        )

    config.START_POSITION = np.array(
        cfg_get(config, "START_POSITION", [0.0, 0.0, 0.0]),
        dtype=np.float64
    )

    config.CONTROL_VARIANCE = np.array(
        cfg_get(config, "CONTROL_VARIANCE", [0.001, 0.0005]),
        dtype=np.float64
    )

    if not hasattr(config, "sensor") or config.sensor is None:
        raise RuntimeError("Config must contain a sensor: block")

    config.sensor.MAX_MEASUREMENTS = int(cfg_get(config.sensor, "MAX_MEASUREMENTS", 50))
    config.sensor.RANGE = float(cfg_get(config.sensor, "RANGE", 15.0))
    config.sensor.MIN_RANGE = float(cfg_get(config.sensor, "MIN_RANGE", 1.5))
    config.sensor.FOV = float(cfg_get(config.sensor, "FOV", 1.22173))
    config.sensor.LIDAR_X_OFFSET = float(cfg_get(config.sensor, "LIDAR_X_OFFSET", 0.52))
    # Distance from the rear axle forward to base_link (the /odom point). The particles move along
    # their heading (no sideslip), which only the rear axle does; see FastSLAMNode.slam_callback.
    config.REAR_AXLE_OFFSET = float(cfg_get(config, "REAR_AXLE_OFFSET", 0.0))
    # weight of a measurement that starts a new landmark (see fastslam64/README.md); 0 = off
    config.NEW_LANDMARK_D2 = float(cfg_get(config, "NEW_LANDMARK_D2", 0.0))
    config.sensor.VARIANCE = np.array(
        cfg_get(config.sensor, "VARIANCE", [0.01, 0.001]),
        dtype=np.float64
    )
    config.sensor.COVARIANCE = np.diag(config.sensor.VARIANCE).astype(np.float64)
    # beyond FAR_RANGE: FAR_VARIANCE (default: the same as VARIANCE, i.e. off)
    config.sensor.FAR_RANGE = float(cfg_get(config.sensor, "FAR_RANGE", 1e9))
    config.sensor.FAR_VARIANCE = np.array(
        cfg_get(config.sensor, "FAR_VARIANCE", list(config.sensor.VARIANCE)), dtype=np.float64
    )

    config.PARTICLES_PER_THREAD = config.N // config.THREADS
    config.PARTICLE_SIZE = 6 + 8 * config.MAX_LANDMARKS

    # Optional endurance block. All tuning defaults live in code to keep YAML small.
    if not hasattr(config, "endurance") or config.endurance is None:
        config.endurance = dotify({})

    config.endurance.ENABLED = bool(cfg_get(config.endurance, "ENABLED", True))
    config.endurance.FREEZE_AFTER_LAPS = int(cfg_get(config.endurance, "FREEZE_AFTER_LAPS", 2))
    config.endurance.MIN_FREEZE_TIME = float(cfg_get(config.endurance, "MIN_FREEZE_TIME", 5.0))
    config.endurance.MIN_LANDMARKS_TO_FREEZE = int(
        cfg_get(config.endurance, "MIN_LANDMARKS_TO_FREEZE", 20)
    )
    # Same-colour landmarks closer than this are one cone mapped twice (data association failed
    # while the particle's pose was off); merged when the map is frozen. 0 = off.
    config.endurance.MERGE_RADIUS = float(cfg_get(config.endurance, "MERGE_RADIUS", 0.7))

    # Hidden/default endurance tuning. You can expose these later only if needed.
    config.endurance.LOCALIZATION_THRESHOLD = float(
        cfg_get(config.endurance, "LOCALIZATION_THRESHOLD", 10.0)
    )
    config.endurance.LOCALIZATION_CONTROL_NOISE_MULTIPLIER = float(
        cfg_get(config.endurance, "LOCALIZATION_CONTROL_NOISE_MULTIPLIER", 3.0)
    )
    config.endurance.MAPPING_RESAMPLE_TRIGGER_RATIO = float(
        cfg_get(config.endurance, "MAPPING_RESAMPLE_TRIGGER_RATIO", 0.3)
    )
    config.endurance.LOCALIZATION_RESAMPLE_TRIGGER_RATIO = float(
        cfg_get(config.endurance, "LOCALIZATION_RESAMPLE_TRIGGER_RATIO", 0.6)
    )

    loc_cov = cfg_get(config.endurance, "LOCALIZATION_COVARIANCE", [0.25, 0.01])
    config.endurance.LOCALIZATION_COVARIANCE = np.array(loc_cov, dtype=np.float64)

    return config, config_path


# ------------------ Utility Helpers ------------------
def make_grid(n_items: int, block_x: int):
    if block_x <= 0:
        raise ValueError("block_x must be > 0")
    grid_x = (n_items + block_x - 1) // block_x
    return (max(1, int(grid_x)), 1, 1)


def ensure_contiguous(arr: np.ndarray, dtype):
    if arr is None:
        return None
    return np.ascontiguousarray(arr, dtype=dtype)


def print_gpu_stats(prefix=""):
    if not DEBUG_GPU:
        return
    free, total = cuda.mem_get_info()
    print(f"[GPU DEBUG] {prefix} | Free: {free/1e6:.2f} MB / Total: {total/1e6:.2f} MB")


def check_and_memcpy_htod(dev_ptr, host_arr, name=""):
    if host_arr is None:
        raise RuntimeError(f"Host array for {name} is None")
    host_arr = ensure_contiguous(host_arr, host_arr.dtype)
    try:
        cuda.memcpy_htod(dev_ptr, host_arr)
        print_gpu_stats(f"HTOD {name}")
    except Exception as e:
        raise RuntimeError(f"memcpy_htod failed for {name}: {e}")


def check_and_memcpy_dtoh(host_arr, dev_ptr, name=""):
    if host_arr is None:
        raise RuntimeError(f"Host target for {name} is None")
    try:
        cuda.memcpy_dtoh(host_arr, dev_ptr)
        print_gpu_stats(f"DTOH {name}")
    except Exception as e:
        raise RuntimeError(f"memcpy_dtoh failed for {name}: {e}")


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def xy_to_range_bearing(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    beta = math.atan2(y, x)
    return r, beta


def safe_cone_type(value):
    """
    Handles scalar or array-like colour/type values from FlatParticle.get_colors().
    This fixes: "only length-1 arrays can be converted to Python scalars".
    """
    arr = np.asarray(value).reshape(-1)
    if arr.size == 0:
        return int(Cone.UNKNOWN)
    try:
        return int(arr[0])
    except Exception:
        return int(Cone.UNKNOWN)


def get_cone_color(cone_type: int):
    color = {"r": 0.0, "g": 0.0, "b": 0.0, "a": 1.0}

    if cone_type == Cone.YELLOW:
        color["r"], color["g"], color["b"] = 1.0, 1.0, 0.0
    elif cone_type == Cone.BLUE:
        color["r"], color["g"], color["b"] = 0.0, 0.0, 1.0
    elif cone_type in [Cone.ORANGE, Cone.LARGE_ORANGE]:
        color["r"], color["g"], color["b"] = 1.0, 0.5, 0.0
    else:
        color["r"], color["g"], color["b"] = 0.5, 0.5, 0.5

    return color


# ------------------ FastSLAM Core ------------------
class FastSLAMCore:
    def __init__(self, config, logger=None):
        self.config = config
        self.logger = logger

        self.N = int(config.N)
        self.THREADS = int(config.THREADS)
        self.MAX_MEASUREMENTS = int(config.sensor.MAX_MEASUREMENTS)

        self.mode = "mapping"
        self.frozen_map = None
        self.frozen_count = 0
        self.frozen_map_data = []
        self.persistent_weights = bool(cfg_get(config, "PERSISTENT_WEIGHTS", True))

        self._info(f"Chosen FastSLAM N: {self.N}")
        self._info(f"Threads: {self.THREADS}")
        self._info(f"Particles per thread: {self.config.PARTICLES_PER_THREAD}")
        self._info(f"Sensor measurement limit: {self.MAX_MEASUREMENTS}")

        self.particles = FlatParticle.get_initial_particles(
            self.N,
            self.config.MAX_LANDMARKS,
            self._rear_axle_start(),
            sigma=0.2
        ).astype(KERN_FLOAT)

        self.cuda_modules = fastslam_cuda.load_cuda_modules(
            THREADS=self.THREADS,
            PARTICLE_SIZE=int(self.config.PARTICLE_SIZE),
            N_PARTICLES=self.N,
            FAR_RANGE=float(self.config.sensor.FAR_RANGE),
            FAR_VAR_R=float(self.config.sensor.FAR_VARIANCE[0]),
            FAR_VAR_B=float(self.config.sensor.FAR_VARIANCE[1]),
            NEW_LM_D2=float(self.config.NEW_LANDMARK_D2),
        )

        self.memory = CUDAMemory(self.config)

        check_and_memcpy_htod(
            self.memory.cov,
            ensure_contiguous(self.config.sensor.COVARIANCE, KERN_FLOAT),
            name="mapping_covariance"
        )
        check_and_memcpy_htod(
            self.memory.particles,
            ensure_contiguous(self.particles, KERN_FLOAT),
            name="particles"
        )

        try:
            init_rng_func = self.cuda_modules["predict"].get_function("init_rng")
            init_rng_func(
                np.int32(self.config.SEED),
                block=(self.THREADS, 1, 1),
                grid=make_grid(self.N, self.THREADS)
            )
            cuda.Context.synchronize()
        except Exception as e:
            raise RuntimeError(f"init_rng failed: {e}")

        self.weights = np.zeros(self.N, dtype=WEIGHT_DTYPE)
        self._info("CUDA FastSLAM mapping core initialised.")

    def _rear_axle_start(self):
        # START_POSITION is the base_link pose; the particles hold the rear axle pose
        x, y, th = self.config.START_POSITION
        L = self.config.REAR_AXLE_OFFSET
        return np.array([x - L * math.cos(th), y - L * math.sin(th), th], dtype=np.float64)

    def _reset_weights(self):
        try:
            reset_weights_func = self.cuda_modules["resample"].get_function("reset_weights")
            reset_weights_func(
                self.memory.particles,
                block=(self.THREADS, 1, 1),
                grid=make_grid(self.N, self.THREADS)
            )
            cuda.Context.synchronize()
        except Exception as e:
            raise RuntimeError(f"reset_weights kernel failed: {e}")

    def _info(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def _warn(self, msg):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f"WARNING: {msg}")

    def _particles_2d(self):
        return self.particles.reshape((self.N, self.config.PARTICLE_SIZE))

    def _force_frozen_map_on_host(self):
        if self.frozen_map is None:
            return
        particles_2d = self._particles_2d()
        particles_2d[:, 5] = self.frozen_count
        particles_2d[:, 6:] = self.frozen_map

    def _map_data_from_block(self, block, n):
        ml = int(self.config.MAX_LANDMARKS)
        means = block[:2 * ml].reshape(ml, 2)[:n]
        covs = block[2 * ml:6 * ml].reshape(ml, 2, 2)[:n]
        probs = block[6 * ml:7 * ml][:n]
        colors = block[7 * ml:8 * ml][:n]
        return [{"position": means[i], "covariance": covs[i], "type": safe_cone_type(colors[i]), "prob": float(probs[i])}
                for i in range(n) if not np.all(covs[i] == 0) and np.linalg.norm(means[i]) > 0.01]

    def _dump_map(self, map_data, n_before):
        # debugging hook: FSAI_MAP_DUMP=/path/file.csv writes the frozen map (x, y, type) and the
        # landmark count before merging
        path = os.environ.get("FSAI_MAP_DUMP")
        if path:
            with open(path, "w") as f:
                f.write(f"# landmarks before merge: {n_before}\nx,y,type,prob\n")
                for lm in map_data:
                    f.write(f"{lm['position'][0]:.3f},{lm['position'][1]:.3f},{lm['type']},{lm['prob']:.1f}\n")

    def _force_frozen_map_on_gpu(self, name):
        if self.frozen_map is None:
            return
        self._force_frozen_map_on_host()
        check_and_memcpy_htod(
            self.memory.particles,
            ensure_contiguous(self.particles, KERN_FLOAT),
            name=name
        )

    def _merge_duplicate_landmarks(self, block, n, radius):
        """Merge same-colour landmarks within `radius` of each other in one particle's landmark block
        (means | covariances | probabilities | colours). Returns (new block, new landmark count)."""
        ml = int(self.config.MAX_LANDMARKS)
        means = block[:2 * ml].reshape(ml, 2)[:n]
        covs = block[2 * ml:6 * ml].reshape(ml, 4)[:n]
        probs = block[6 * ml:7 * ml][:n]
        colors = block[7 * ml:8 * ml][:n]
        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for i in range(n):
            d = np.hypot(means[i + 1:, 0] - means[i, 0], means[i + 1:, 1] - means[i, 1])
            for j in np.nonzero((d < radius) & (colors[i + 1:] == colors[i]))[0] + i + 1:
                parent[find(int(j))] = find(i)

        groups = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(i)
        out = np.zeros_like(block)
        o_means = out[:2 * ml].reshape(ml, 2)
        o_covs = out[2 * ml:6 * ml].reshape(ml, 4)
        o_probs, o_colors = out[6 * ml:7 * ml], out[7 * ml:8 * ml]
        for k, members in enumerate(sorted(groups.values(), key=min)):
            best = min(members, key=lambda m: covs[m, 0] + covs[m, 3])   # tightest covariance
            o_means[k] = means[members].mean(axis=0)
            o_covs[k], o_probs[k], o_colors[k] = covs[best], probs[best], colors[best]
        return out, len(groups)

    def freeze_best_map(self, final_map_data, elapsed_time, lap_count):
        if self.frozen_map is not None:
            return False

        min_landmarks = int(self.config.endurance.MIN_LANDMARKS_TO_FREEZE)
        if len(final_map_data) < min_landmarks:
            self._warn(
                f"Freeze rejected: only {len(final_map_data)} landmarks, "
                f"minimum required is {min_landmarks}."
            )
            return False

        try:
            # Use the same API pattern as your previous working node.
            best_idx = int(np.argmax(FlatParticle.w(self.particles)))
            particles_2d = self._particles_2d()
            self.frozen_map = np.copy(particles_2d[best_idx, 6:])
            self.frozen_count = int(particles_2d[best_idx, 5])
            n_before = self.frozen_count
            radius = float(self.config.endurance.MERGE_RADIUS)
            if radius > 0.0:
                self.frozen_map, self.frozen_count = self._merge_duplicate_landmarks(
                    self.frozen_map, self.frozen_count, radius)
                if self.frozen_count < n_before:
                    self._warn(f"Merged {n_before - self.frozen_count} duplicate landmarks "
                               f"({n_before} -> {self.frozen_count}, radius {radius} m).")
            self.frozen_map_data = self._map_data_from_block(self.frozen_map, self.frozen_count)
            self._dump_map(self.frozen_map_data, n_before)
        except Exception as e:
            raise RuntimeError(f"Failed to freeze best particle map: {e}")

        # Switch covariance used by CUDA update to smoother localisation covariance.
        loc_cov = np.diag(self.config.endurance.LOCALIZATION_COVARIANCE).astype(KERN_FLOAT)
        check_and_memcpy_htod(
            self.memory.cov,
            ensure_contiguous(loc_cov, KERN_FLOAT),
            name="localization_covariance"
        )

        self.mode = "localization"
        self._force_frozen_map_on_gpu(name="freeze_best_map")
        final_map_data[:] = self.frozen_map_data

        self._warn("=" * 72)
        self._warn(
            f"MAP FROZEN: t={elapsed_time:.2f}s, /laps={lap_count}, "
            f"visible_landmarks={len(final_map_data)}"
        )
        self._warn("Mode changed from mapping to localization/frozen-map mode.")
        self._warn("=" * 72)

        return True

    def execute_step(
        self,
        v_linear: float,
        omega_angular: float,
        measurements_rb: np.ndarray,
        dt: float,
        elapsed_time: float,
        lap_count: int,
        freeze_requested: bool,
    ):
        if measurements_rb is None or len(measurements_rb) == 0:
            measurements_rb = np.zeros((0, 3), dtype=KERN_FLOAT)
        else:
            if len(measurements_rb) > self.MAX_MEASUREMENTS:
                measurements_rb = measurements_rb[:self.MAX_MEASUREMENTS]
            measurements_rb = ensure_contiguous(np.array(measurements_rb), KERN_FLOAT)

        n_measurements = len(measurements_rb)

        # In frozen-map mode, force map before prediction/update.
        if self.frozen_map is not None:
            self.mode = "localization"
            self._force_frozen_map_on_gpu(name="pre_step_frozen_map")

        # Weights. Sequential importance resampling: a particle's weight is the product of its
        # measurement likelihoods since the last resample (rescale() normalises them each step),
        # and is reset only after resampling. Resetting every step (persistent_weights:=false, the
        # old behaviour) made resampling see one step's evidence only, so particles whose maps had
        # drifted survived and the published map filled with duplicates as soon as the process
        # noise was > 0 (sim, perfect odometry: ~125 landmarks for 72 cones; with this ~80-100).
        if not self.persistent_weights:
            self._reset_weights()

        # Mapping vs frozen-map localisation parameters.
        if self.frozen_map is None:
            control_noise_multiplier = 1.0
            update_threshold = float(self.config.THRESHOLD)
            resample_trigger = float(self.config.endurance.MAPPING_RESAMPLE_TRIGGER_RATIO) * self.N
        else:
            control_noise_multiplier = float(self.config.endurance.LOCALIZATION_CONTROL_NOISE_MULTIPLIER)
            update_threshold = float(self.config.endurance.LOCALIZATION_THRESHOLD)
            resample_trigger = float(self.config.endurance.LOCALIZATION_RESAMPLE_TRIGGER_RATIO) * self.N

        # Prediction. The kernel turns first and then moves along the OLD heading (Euler), which
        # biases every corner inwards. Two half-steps (turn half, then move + turn the other
        # half) move the particles along the mid-step heading instead.
        try:
            prediction_func = self.cuda_modules["predict"].get_function("predict_from_model")
            # sigmas scaled so the summed noise over both half-steps equals one full step
            sigma_w = np.sqrt(2.0 * float(self.config.CONTROL_VARIANCE[1]) * control_noise_multiplier)
            sigma_v = np.sqrt(float(self.config.CONTROL_VARIANCE[0]) * control_noise_multiplier)
            half_dt = np.float64(dt / 2.0)
            for v_step, sigma_v_step in ((0.0, 0.0), (2.0 * v_linear, 2.0 * sigma_v)):
                prediction_func(
                    self.memory.particles,
                    np.float64(omega_angular),
                    np.float64(v_step),
                    np.float64(sigma_w),
                    np.float64(sigma_v_step),
                    half_dt,
                    block=(self.THREADS, 1, 1),
                    grid=make_grid(self.N, self.THREADS)
                )
            cuda.Context.synchronize()
        except Exception as e:
            raise RuntimeError(f"predict_from_model failed: {e}")

        # Update.
        if n_measurements > 0:
            try:
                check_and_memcpy_htod(self.memory.measurements, measurements_rb, name="measurements")
                update_func = self.cuda_modules["update"].get_function("update")
                update_func(
                    self.memory.particles,
                    np.int32(self.N // self.THREADS),
                    self.memory.scratchpad,
                    np.int32(self.memory.scratchpad_block_size),
                    self.memory.measurements,
                    np.int32(self.N),
                    np.int32(n_measurements),
                    self.memory.cov,
                    np.float64(update_threshold),
                    np.float64(self.config.sensor.RANGE),
                    np.float64(self.config.sensor.FOV),
                    np.int32(self.config.MAX_LANDMARKS),
                    block=(self.THREADS, 1, 1),
                    grid=make_grid(self.N, self.THREADS)
                )
                cuda.Context.synchronize()
            except Exception as e:
                raise RuntimeError(f"update kernel or HtoD failed: {e}\n{traceback.format_exc()}")

        # Critical endurance part:
        # The CUDA update may internally create/update landmarks. In frozen mode,
        # immediately overwrite landmark memory with the frozen best-particle map.
        if self.frozen_map is not None:
            check_and_memcpy_dtoh(self.particles, self.memory.particles, name="particles_after_update")
            self._force_frozen_map_on_gpu(name="post_update_frozen_map")

        # Rescale and pose estimate.
        try:
            rescale(self.cuda_modules, self.config, self.memory)
            estimated_pose = get_pose_estimate(self.cuda_modules, self.config, self.memory)
            cuda.Context.synchronize()
        except Exception as e:
            raise RuntimeError(f"get_pose_estimate failed: {e}")

        # Compute Neff and resample.
        try:
            weights_func = self.cuda_modules["weights_and_mean"].get_function("get_weights")
            weights_func(
                self.memory.particles,
                self.memory.weights,
                block=(self.THREADS, 1, 1),
                grid=make_grid(self.N, self.THREADS)
            )
            cuda.Context.synchronize()

            check_and_memcpy_dtoh(self.weights, self.memory.weights, name="weights")
            neff = FlatParticle.neff(self.weights.astype(np.float64))

            if neff < resample_trigger:
                resample(self.cuda_modules, self.config, self.weights, self.memory, 0.5)
                cuda.Context.synchronize()
                if self.persistent_weights:
                    self._reset_weights()

                if self.frozen_map is not None:
                    check_and_memcpy_dtoh(
                        self.particles,
                        self.memory.particles,
                        name="particles_after_resample"
                    )
                    self._force_frozen_map_on_gpu(name="post_resample_frozen_map")

        except Exception as e:
            raise RuntimeError(f"weights_and_mean / resample failed: {e}")

        # Extract best map using your previous working method.
        try:
            check_and_memcpy_dtoh(self.particles, self.memory.particles, name="particles")
            best_idx = int(np.argmax(FlatParticle.w(self.particles)))
            best_landmarks = FlatParticle.get_landmarks(self.particles, best_idx)
            # Weighted circular mean of the heading: a linear mean of particles at +3.1 and -3.1 rad
            # (driving west) is ~0 and flipped the published heading by up to 180 deg. The vendored
            # get_mean_position kernel does the same; this host copy keeps the pose on one array.
            p2d = self._particles_2d()
            w = p2d[:, 3] / max(p2d[:, 3].sum(), 1e-300)
            estimated_pose = np.array([
                np.dot(w, p2d[:, 0]), np.dot(w, p2d[:, 1]),
                math.atan2(np.dot(w, np.sin(p2d[:, 2])), np.dot(w, np.cos(p2d[:, 2]))),
            ])
            best_covariances = FlatParticle.get_covariances(self.particles, best_idx)
            best_colors = FlatParticle.get_colors(self.particles, best_idx)
        except Exception as e:
            raise RuntimeError(f"postprocessing particles failed: {e}")

        final_map_data = []
        limit = min(len(best_landmarks), len(best_covariances), len(best_colors))

        for i in range(limit):
            lm = best_landmarks[i]
            cov = best_covariances[i]
            color_val = best_colors[i]

            # Same filtering as old code, with safer type conversion.
            if not np.all(cov == 0) and np.linalg.norm(lm[:2]) > 0.01:
                final_map_data.append({
                    "position": lm,
                    "covariance": cov,
                    "type": safe_cone_type(color_val),
                })

        # Trigger freeze only after we have a real map from this step.
        if (
            freeze_requested
            and self.config.endurance.ENABLED
            and self.frozen_map is None
            and elapsed_time >= float(self.config.endurance.MIN_FREEZE_TIME)
        ):
            self.freeze_best_map(final_map_data, elapsed_time, lap_count)

        return estimated_pose, final_map_data, self.mode


# ------------------ ROS 2 Node ------------------
class FastSLAMNode(Node):
    def __init__(self):
        super().__init__("fastslam_node")

        # Default True because your current work is simulation-focused.
        use_sim = self.declare_parameter("use_sim", True).value
        self.config, config_path = load_config(use_sim)

        self.get_logger().info(f"Loaded config: {config_path}")
        self.get_logger().info("Stable endurance FastSLAM node initialising...")

        self.MAX_MEASUREMENTS = int(self.config.sensor.MAX_MEASUREMENTS)

        self.last_time = None
        self.last_odom_pose = None
        self.rear_axle_offset = float(self.config.REAR_AXLE_OFFSET)
        self.motion_from_odom_pose = bool(self.declare_parameter("motion_from_odom_pose", True).value)
        # after the map freeze: localise by colour-aware cone matching (false: keep the particle filter)
        self.match_localisation = bool(self.declare_parameter("match_localisation", True).value)
        self.match_gate = float(self.declare_parameter("match_gate", 1.0).value)    # m
        self.match_gain = float(self.declare_parameter("match_gain", 0.5).value)    # blend per update
        self.match_max_step = float(self.declare_parameter("match_max_step", 0.15).value)  # m per update
        self.match_max_turn = float(self.declare_parameter("match_max_turn", 0.3).value)  # deg per update
        self.loc_map = None
        self.loc_map_data = None
        self.loc_correction = None
        self.loc_matches = 0
        self.start_time = None
        self.latest_lap_count = 0
        self.last_status_time = None
        self.last_mode = "mapping"

        self.enable_status_logging = bool(
            self.declare_parameter("enable_status_logging", True).value
        )
        self.status_log_period = float(
            self.declare_parameter("status_log_period", 1.0).value
        )

        self.slam_core = FastSLAMCore(self.config, logger=self.get_logger())
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers.
        self.pose_pub = self.create_publisher(PoseStamped, "/slam/pose", 1)
        self.cone_map_pub = self.create_publisher(ConeArray, "/slam/cone_map", 1)
        self.map_markers_pub = self.create_publisher(MarkerArray, "/slam/cone_map_markers", 1)
        # Nearby mapped cones as obstacles for the MPPI controller, in its frame (odom).
        self.obstacles_pub = self.create_publisher(PointCloud2, "/slam/cones", 1)
        self.obstacle_radius = float(self.declare_parameter("obstacle_radius", 25.0).value)

        # Subscribers.
        self.odom_sub = Subscriber(self, Odometry, "/odom")
        self.cone_sub = Subscriber(self, ConeArray, "/cone_array")
        self.lap_sub = self.create_subscription(UInt16, "/laps", self.lap_callback, 10)

        # /odom arrives at ~100 Hz but /cone_array only after YOLO + clustering, so the odom
        # queue must cover that latency (20 msgs was ~0.1 s at sim rate: frequent misses and
        # "Abnormal dt" skips).
        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.cone_sub],
            queue_size=int(self.declare_parameter("sync_queue_size", 150).value),
            slop=0.1
        )
        self.ts.registerCallback(self.slam_callback)

        self.get_logger().info("Waiting for /odom, /cone_array, and /laps...")
        self.get_logger().info(
            "Config summary: "
            f"freeze_after_laps={self.config.endurance.FREEZE_AFTER_LAPS}, "
            f"min_freeze_time={self.config.endurance.MIN_FREEZE_TIME}, "
            f"min_landmarks={self.config.endurance.MIN_LANDMARKS_TO_FREEZE}"
        )

    def lap_callback(self, msg: UInt16):
        new_lap_count = int(msg.data)
        if new_lap_count != self.latest_lap_count:
            self.get_logger().warn(f"/laps changed: {self.latest_lap_count} -> {new_lap_count}")
        self.latest_lap_count = new_lap_count

    def slam_callback(self, odom_msg: Odometry, cone_array_msg: ConeArray):
        # Odometry input.
        v_linear = float(odom_msg.twist.twist.linear.x)
        omega_angular = float(odom_msg.twist.twist.angular.z)

        rx = float(odom_msg.pose.pose.position.x)
        ry = float(odom_msg.pose.pose.position.y)
        rq = odom_msg.pose.pose.orientation
        r_yaw = quaternion_to_yaw(rq.x, rq.y, rq.z, rq.w)

        # Measurements.
        measurements_rb = self._process_cones(cone_array_msg, rx, ry, r_yaw)

        # Time.
        current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = current_time

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        if self.motion_from_odom_pose:
            # Motion input from the odom pose change since the last APPLIED step, not the
            # instantaneous twist. Integrating one twist sample per ~0.1 s step, and dropping
            # the motion of every skipped frame, drifted the map by metres per lap. Measuring
            # from the last applied pose means skipped frames lose nothing, and the particles
            # move and turn exactly as much as odometry did.
            if self.last_odom_pose is None:
                self.last_odom_pose = (current_time, rx, ry, r_yaw)
                return
            t0, x0, y0, yaw0 = self.last_odom_pose
            dt = current_time - t0
            if dt <= 0.0:
                return  # out-of-order pair from the synchroniser; nothing to apply
            # The particle motion model moves along the heading, as only the rear axle does:
            # base_link (the car's centre) also moves sideways at yaw rate x REAR_AXLE_OFFSET, and
            # projecting its motion on the heading dropped that every step, so the particles traced
            # the rear axle's path while measurements were placed from base_link (up to 1.5 m apart
            # after a half turn). So the particles hold the REAR AXLE pose (rear-relative cones,
            # START_POSITION shifted back), converted back to base_link for every output.
            L = self.rear_axle_offset
            dx = (rx - L * math.cos(r_yaw)) - (x0 - L * math.cos(yaw0))
            dy = (ry - L * math.sin(r_yaw)) - (y0 - L * math.sin(yaw0))
            dyaw = math.atan2(math.sin(r_yaw - yaw0), math.cos(r_yaw - yaw0))
            mid_yaw = yaw0 + dyaw / 2.0
            v_linear = (dx * math.cos(mid_yaw) + dy * math.sin(mid_yaw)) / dt
            omega_angular = dyaw / dt

        if abs(v_linear) < 0.03 and abs(omega_angular) < 0.02 and len(measurements_rb) == 0:
            return

        if not self.motion_from_odom_pose and (dt <= 0.0 or dt > 0.5):
            self.get_logger().warn(f"Abnormal dt detected: {dt:.4f}s. Skipping frame.")
            return

        if abs(v_linear) < 1e-9 and abs(omega_angular) < 1e-9 and len(measurements_rb) == 0:
            return

        elapsed_time = current_time - self.start_time

        freeze_requested = (
            self.config.endurance.ENABLED
            and self.slam_core.frozen_map is None
            and self.latest_lap_count >= int(self.config.endurance.FREEZE_AFTER_LAPS)
        )

        if self.loc_map is not None:
            # frozen map: colour-aware cone matching instead of the particle filter
            estimated_pose = self._match_localise(measurements_rb, rx, ry, r_yaw)
            final_map_data, mode = self.loc_map_data, "localization"
        else:
            try:
                estimated_pose, final_map_data, mode = self.slam_core.execute_step(
                    v_linear=v_linear,
                    omega_angular=omega_angular,
                    measurements_rb=self._to_rear_axle(measurements_rb),
                    dt=dt,
                    elapsed_time=elapsed_time,
                    lap_count=self.latest_lap_count,
                    freeze_requested=freeze_requested,
                )
            except Exception as e:
                self.get_logger().error(f"SLAM/localisation execution error: {e}")
                self.get_logger().debug(traceback.format_exc())
                return
            L = self.rear_axle_offset
            estimated_pose = np.array([estimated_pose[0] + L * math.cos(estimated_pose[2]),
                                       estimated_pose[1] + L * math.sin(estimated_pose[2]),
                                       estimated_pose[2]])
            if self.match_localisation and self.slam_core.frozen_map is not None:
                self._start_match_localisation(final_map_data, estimated_pose, rx, ry, r_yaw)
        self.last_odom_pose = (current_time, rx, ry, r_yaw)

        # Publish outputs.
        self._publish_transforms(odom_msg, estimated_pose)
        self._publish_pose_stamped(estimated_pose, odom_msg.header.stamp)
        self._publish_cone_map(final_map_data, odom_msg.header.stamp)
        self._publish_map_markers(final_map_data, odom_msg.header.stamp)
        self._publish_obstacles(final_map_data, odom_msg, estimated_pose)

        self._status_log(current_time, elapsed_time, mode, estimated_pose, len(final_map_data), len(measurements_rb))

    def _to_rear_axle(self, measurements_rb):
        # [range, bearing, type] from base_link -> from the rear axle
        L = self.rear_axle_offset
        if L == 0.0 or len(measurements_rb) == 0:
            return measurements_rb
        m = measurements_rb.copy()
        x = m[:, 0] * np.cos(m[:, 1]) + L
        y = m[:, 0] * np.sin(m[:, 1])
        m[:, 0], m[:, 1] = np.hypot(x, y), np.arctan2(y, x)
        return m

    # ---- Localisation on the frozen map by colour-aware cone matching -----------------------
    # The particle filter's data association ignores cone colour and gates at a fixed distance.
    # Once the pose is ~1 m off, cones match the wrong landmarks, which pulls it further off, and
    # beyond the gate nothing matches and it never recovers: in sim it diverged by 4-8 m a few
    # laps after the freeze. With the map fixed, localisation is a small alignment problem: match
    # each cone to the nearest frozen landmark OF THE SAME COLOUR and fit the rigid transform.

    def _start_match_localisation(self, final_map_data, estimated_pose, rx, ry, r_yaw):
        self.loc_map_data = list(final_map_data)
        self.loc_map = np.array([[lm["position"][0], lm["position"][1], lm["type"]] for lm in final_map_data])
        self.loc_correction = self._correction(estimated_pose, (rx, ry, r_yaw))
        self.get_logger().warn(f"localisation by colour-aware cone matching on {len(self.loc_map)} frozen landmarks")

    @staticmethod
    def _correction(map_pose, odom_pose):
        # T such that map_pose = T (+) odom_pose
        dyaw = map_pose[2] - odom_pose[2]
        c, s = math.cos(dyaw), math.sin(dyaw)
        return np.array([map_pose[0] - (c * odom_pose[0] - s * odom_pose[1]),
                         map_pose[1] - (s * odom_pose[0] + c * odom_pose[1]), dyaw])

    @staticmethod
    def _apply(corr, odom_pose):
        c, s = math.cos(corr[2]), math.sin(corr[2])
        return np.array([corr[0] + c * odom_pose[0] - s * odom_pose[1],
                         corr[1] + s * odom_pose[0] + c * odom_pose[1],
                         math.atan2(math.sin(corr[2] + odom_pose[2]), math.cos(corr[2] + odom_pose[2]))])

    def _match_localise(self, measurements_rb, rx, ry, r_yaw):
        pose = self._apply(self.loc_correction, (rx, ry, r_yaw))
        self.loc_matches = 0
        if len(measurements_rb) < 3:
            return pose
        r, b, kind = measurements_rb[:, 0], measurements_rb[:, 1], measurements_rb[:, 2]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        lx, ly = r * np.cos(b), r * np.sin(b)
        meas = np.column_stack((pose[0] + c * lx - s * ly, pose[1] + s * lx + c * ly))
        src, dst = [], []
        for i, (p, k) in enumerate(zip(meas, kind)):
            same = self.loc_map[self.loc_map[:, 2] == k]
            if k == Cone.UNKNOWN or len(same) == 0:
                continue
            d = np.hypot(same[:, 0] - p[0], same[:, 1] - p[1])
            j = int(np.argmin(d))
            if d[j] >= self.match_gate:
                continue
            # mutual nearest neighbour: no other measured cone of this colour is closer to that
            # landmark (along an evenly spaced edge a cone can otherwise match its neighbour)
            others = meas[(kind == k) & (np.arange(len(meas)) != i)]
            if len(others) and np.min(np.hypot(others[:, 0] - same[j, 0], others[:, 1] - same[j, 1])) < d[j]:
                continue
            src.append(p)
            dst.append(same[j, :2])
        self.loc_matches = len(src)
        if len(src) < 3:
            return pose
        src, dst = np.array(src), np.array(dst)
        ms, md = src.mean(axis=0), dst.mean(axis=0)
        # A few cones in a line (one track edge) don't determine the rotation: then translation only.
        spread = np.linalg.svd(src - ms, compute_uv=False)[-1] / math.sqrt(len(src))
        dth = 0.0
        if spread > 1.0:
            u, _, vt = np.linalg.svd((src - ms).T @ (dst - md))
            rot = (u @ vt).T
            if np.linalg.det(rot) > 0:
                dth = math.atan2(rot[1, 0], rot[0, 0])
        # rotate about the matched cones' centre, then shift onto the landmarks
        c, s = math.cos(dth), math.sin(dth)
        rel = pose[:2] - ms
        new_xy = md + np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]])
        # odometry is good, so real corrections are small: limit each step, so that a wrong match
        # can't drag the pose away (it then also needs many consistent updates to move it)
        step = self.match_gain * (new_xy - pose[:2])
        n = np.hypot(*step)
        if n > self.match_max_step:
            step *= self.match_max_step / n
        max_turn = math.radians(self.match_max_turn)
        pose = np.array([pose[0] + step[0], pose[1] + step[1],
                         pose[2] + float(np.clip(self.match_gain * dth, -max_turn, max_turn))])
        self.loc_correction = self._correction(pose, (rx, ry, r_yaw))
        return pose

    def _status_log(self, current_time, elapsed_time, mode, estimated_pose, landmark_count, measurement_count):
        if not self.enable_status_logging:
            return

        if self.last_status_time is None:
            self.last_status_time = current_time

        if (current_time - self.last_status_time) < self.status_log_period:
            return

        self.last_status_time = current_time

        x = float(estimated_pose[0])
        y = float(estimated_pose[1])
        yaw = float(estimated_pose[2])

        self.get_logger().info(
            f"mode={mode}, laps={self.latest_lap_count}, "
            f"t={elapsed_time:.1f}s, pose=({x:.2f}, {y:.2f}, {yaw:.2f}), "
            f"measurements={measurement_count}, landmarks={landmark_count}"
            + (f", matched={self.loc_matches}, correction=({self.loc_correction[0]:.2f}, "
               f"{self.loc_correction[1]:.2f}, {math.degrees(self.loc_correction[2]):.1f} deg)"
               if self.loc_map is not None else "")
        )

        if mode != self.last_mode:
            self.get_logger().warn(f"SLAM mode changed: {self.last_mode} -> {mode}")
            self.last_mode = mode

    def _process_cones(self, cone_array_msg: ConeArray, rx, ry, r_yaw):
        """
        Extract cones as [range, bearing, cone_type].

        If /cone_array is in map frame, convert map -> base_link using odom pose.
        If /cone_array is in sensor frame, use LIDAR_X_OFFSET exactly like the old code,
        but read it from YAML instead of hard-coding.
        """
        all_cones_rb = []

        is_map_frame = (cone_array_msg.header.frame_id == "map")
        lidar_x_offset = float(self.config.sensor.LIDAR_X_OFFSET)
        max_range = float(self.config.sensor.RANGE)
        min_range = float(self.config.sensor.MIN_RANGE)

        cone_mapping = [
            (cone_array_msg.yellow_cones, Cone.YELLOW),
            (cone_array_msg.blue_cones, Cone.BLUE),
            (cone_array_msg.orange_cones, Cone.ORANGE),
            (cone_array_msg.large_orange_cones, Cone.LARGE_ORANGE),
            (cone_array_msg.unknown_cones, Cone.UNKNOWN),
        ]

        for cone_list, cone_type in cone_mapping:
            for cone in cone_list:
                cx = float(cone.position.x)
                cy = float(cone.position.y)
                if not math.isfinite(cx) or not math.isfinite(cy):
                    continue
                # Reject fake detector outputs at exactly zero.
                if abs(cx) < 1e-6 and abs(cy) < 1e-6:
                    continue

                if is_map_frame:
                    dx = cx - rx
                    dy = cy - ry

                    cos_yaw = math.cos(r_yaw)
                    sin_yaw = math.sin(r_yaw)

                    local_x = dx * cos_yaw + dy * sin_yaw
                    local_y = -dx * sin_yaw + dy * cos_yaw
                else:
                    local_x = cx + lidar_x_offset
                    local_y = cy

                r, beta = xy_to_range_bearing(local_x, local_y)

                if r > max_range or r < min_range:
                    continue

                all_cones_rb.append([r, beta, float(cone_type)])

        measurements_rb = np.array(all_cones_rb, dtype=KERN_FLOAT)

        if len(measurements_rb) > 0:
            sorted_indices = np.argsort(measurements_rb[:, 0])
            measurements_rb = measurements_rb[sorted_indices]

            if len(measurements_rb) > self.MAX_MEASUREMENTS:
                measurements_rb = measurements_rb[:self.MAX_MEASUREMENTS]

        return measurements_rb

    def _publish_transforms(self, odom_msg: Odometry, estimated_pose: np.ndarray):
        x_map = float(estimated_pose[0])
        y_map = float(estimated_pose[1])
        theta_map = float(estimated_pose[2])

        odom_x = float(odom_msg.pose.pose.position.x)
        odom_y = float(odom_msg.pose.pose.position.y)
        odom_q = odom_msg.pose.pose.orientation
        odom_theta = quaternion_to_yaw(odom_q.x, odom_q.y, odom_q.z, odom_q.w)

        delta_theta = theta_map - odom_theta

        t_x = x_map - (odom_x * math.cos(delta_theta) - odom_y * math.sin(delta_theta))
        t_y = y_map - (odom_x * math.sin(delta_theta) + odom_y * math.cos(delta_theta))

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = t_x
        t.transform.translation.y = t_y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(delta_theta)

        self.tf_broadcaster.sendTransform(t)

    def _publish_pose_stamped(self, estimated_pose: np.ndarray, timestamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = float(estimated_pose[0])
        pose_msg.pose.position.y = float(estimated_pose[1])
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(float(estimated_pose[2]))

        self.pose_pub.publish(pose_msg)

    def _publish_cone_map(self, final_map_data, stamp):
        msg = ConeArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"

        for lm in final_map_data:
            cone = Cone()
            cone.position.x = float(lm["position"][0])
            cone.position.y = float(lm["position"][1])
            cone.position.z = 0.15
            cone.type = int(lm["type"])

            if cone.type == Cone.YELLOW:
                msg.yellow_cones.append(cone)
            elif cone.type == Cone.BLUE:
                msg.blue_cones.append(cone)
            elif cone.type == Cone.ORANGE:
                msg.orange_cones.append(cone)
            elif cone.type == Cone.LARGE_ORANGE:
                msg.large_orange_cones.append(cone)
            else:
                msg.unknown_cones.append(cone)

        self.cone_map_pub.publish(msg)

    def _publish_obstacles(self, final_map_data, odom_msg: Odometry, estimated_pose: np.ndarray):
        """Publish mapped cones near the car on /slam/cones (PointCloud2, odom frame).

        Same map -> base_link -> odom chain the pathfinder uses for /path, so obstacles and
        path line up for MPPI.
        """
        if final_map_data:
            pts = np.array([lm["position"][:2] for lm in final_map_data], dtype=np.float64)
        else:
            pts = np.empty((0, 2))

        rx, ry, r_yaw = (float(v) for v in estimated_pose[:3])
        dx, dy = pts[:, 0] - rx, pts[:, 1] - ry
        c, s = math.cos(r_yaw), math.sin(r_yaw)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c
        near = np.hypot(local_x, local_y) < self.obstacle_radius
        local_x, local_y = local_x[near], local_y[near]

        ox = float(odom_msg.pose.pose.position.x)
        oy = float(odom_msg.pose.pose.position.y)
        oq = odom_msg.pose.pose.orientation
        o_yaw = quaternion_to_yaw(oq.x, oq.y, oq.z, oq.w)
        c, s = math.cos(o_yaw), math.sin(o_yaw)
        xyz = np.column_stack((
            ox + local_x * c - local_y * s,
            oy + local_x * s + local_y * c,
            np.zeros_like(local_x),
        )).astype(np.float32)

        header = Header()
        header.stamp = odom_msg.header.stamp
        header.frame_id = "odom"
        self.obstacles_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz.tolist()))

    def _publish_map_markers(self, final_map_data, stamp):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = stamp
        delete_marker.ns = "cone_map_markers"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, lm in enumerate(final_map_data):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "cone_map_markers"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = float(lm["position"][0])
            marker.pose.position.y = float(lm["position"][1])
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5

            color = get_cone_color(int(lm["type"]))
            marker.color.a = color["a"]
            marker.color.r = color["r"]
            marker.color.g = color["g"]
            marker.color.b = color["b"]

            marker_array.markers.append(marker)

        self.map_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FastSLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAM node shut down by user.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()