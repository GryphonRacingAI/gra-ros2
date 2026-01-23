#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import traceback

from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from common_msgs.msg import ConeArray, Cone

import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.autoinit import context
from pycuda._driver import LogicError as CudaLogicError
DEBUG_GPU = True  # Set to False to reduce console spam

# ------------------ FastSLAM Core Imports ------------------
try:
    from fastslam64.lib.particle3 import FlatParticle
    from fastslam64.lib.common import CUDAMemory, rescale, get_pose_estimate, resample
    from fastslam64.cuda.fastslam import load_cuda_modules
    from fastslam64.lib.utils import dotify
except Exception as e:
    raise SystemExit(f"Failed to import FastSLAM core components: {e}")

# TODO: move to a yaml
config = {
    "SEED": 7,
    "N": 512,
    "DT": 1.0,
    "THREADS": 128,
    "GPU_HEAP_SIZE_BYTES": 500 * 1024 * 1024,
    "THRESHOLD": 2.3,
    "sensor": {
        "RANGE": 35,    # Stereo camera max range
        "FOV": 0.7*np.pi,   # FoV i.e 100°
        "MISS_PROB": 0.05,
        "VARIANCE": [0.15**2, np.deg2rad(1.0)**2],
        "MAX_MEASUREMENTS": 50
    },
    # MOTION MODEL (ODOMETRY)
    # Tuned based on your ZED2i /odom and /imu topics.
    # ZED reports very low covariance (1e-9), but we must keep some variance 
    # for the Particle Filter to work (otherwise all particles die).
    
    # Index 0: Rotation Noise (Angular Z)
    #   IMU says cov is ~0.0002. We use ~0.0003 (1.0 degree) to allow for wheel slip.
    #   Previous Sim Value: 5.0 degrees
    #   New Real Value: 1 degree
    
    # Index 1: Linear Noise (Linear X)
    #   ZED Odom is very precise. We reduce linear error to 5cm.
    #   Previous Sim Value: 0.15m
    #   New Real Value: 0.05m
    "CONTROL_VARIANCE": [np.deg2rad(0.0) ** 2, 0.00 ** 2],

    #"GROUND_TRUTH": np.load(os.path.join(cu_dir,"simulation/odom.npy")).astype(np.float64),
    #"CONTROL": np.load(os.path.join(cu_dir,"simulation/control.npy")).astype(np.float64),
    #"LANDMARKS": np.load(os.path.join(cu_dir,"simulation/landmarks.npy")).astype(np.float64), # landmark positions
    "MAX_LANDMARKS": 600,  # upper bound on the total number of landmarks in the environment
    "START_POSITION": np.array([0, 0, 0], dtype=np.float64)         # Will be overwritten by first Odom msg 
}

config = dotify(config)

config.sensor.COVARIANCE = np.diag(config.sensor.VARIANCE).astype(np.float64)
config.PARTICLES_PER_THREAD = config.N // config.THREADS
config.PARTICLE_SIZE = 6 + 8*config.MAX_LANDMARKS

# ---------- Helpers ----------
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


KERN_FLOAT = np.float64
WEIGHT_DTYPE = np.float64
INT_DTYPE = np.int64


# ---------- Utility Functions ----------
def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x , q.y = 0.0, 0.0
    q.z = math.sin(yaw*0.5)
    q.w = math.cos(yaw*0.5)
    return q


def xy_to_range_bearing(x, y):
    r = math.sqrt(x**2 + y**2)
    beta = math.atan2(y, x)
    return r, beta

def get_cone_color(cone_type: int):
    color = {"r":0.0, "g":0.0, "b":0.0, "a":1.0}
    if cone_type == Cone.YELLOW:
        color["r"], color["g"], color["b"] = 1.0, 1.0, 0.0
    elif cone_type == Cone.BLUE:
        color["r"], color["g"], color["b"] = 0.0, 0.0, 1.0
    elif cone_type in [Cone.ORANGE, Cone.LARGE_ORANGE]:
        color["r"], color["g"], color["b"] = 1.0, 0.5, 0.0
    else:
        # Default to Gray for Unknown
        color["r"], color["g"], color["b"] = 0.5, 0.5, 0.5
    return color


# ---------- FastSLAM Core ----------
class FastSLAMCore:
    def __init__(self, config):
        self.config = config
        self.N = int(config.N)
        self.THREADS = int(config.THREADS)
        print(f'choosen N: {self.N}')

        # Handle case where config is missing the variable or it is None
        self.MAX_MEASUREMENTS = config.sensor.MAX_MEASUREMENTS
        print(f"Sensor measurement limit set to: {self.MAX_MEASUREMENTS}")

        self.particles = FlatParticle.get_initial_particles(
            self.N, config.MAX_LANDMARKS, config.START_POSITION.copy(), sigma=0.2
        ).astype(KERN_FLOAT)
        print(f"Initial Partciles loaded")

        self.cuda_modules = load_cuda_modules(
            THREADS=self.THREADS,
            PARTICLE_SIZE=int(config.PARTICLE_SIZE),
            N_PARTICLES=self.N
        )
        print(f"Cuda Modules loaded")

        self.memory = CUDAMemory(config)
        check_and_memcpy_htod(self.memory.cov, ensure_contiguous(config.sensor.COVARIANCE, KERN_FLOAT), name="covariance")
        check_and_memcpy_htod(self.memory.particles, ensure_contiguous(self.particles, KERN_FLOAT), name="particles")
        print(f"Data moved from GPU successful")

        try:
            init_rng_func = self.cuda_modules["predict"].get_function("init_rng")
            grid = make_grid(self.N, self.THREADS)
            init_rng_func(np.int32(config.SEED), block=(self.THREADS,1,1), grid=grid)
            cuda.Context.synchronize()
            print_gpu_stats("RNG init")
        except Exception as e:
            raise RuntimeError(f"init_rng failed: {e}")

        self.weights = np.zeros(self.N, dtype=WEIGHT_DTYPE)

    def execute_step(self, v_linear: float, omega_angular: float, measurements_rb: np.ndarray, dt: float):
        if measurements_rb is None or len(measurements_rb) == 0:
            measurements_rb = np.zeros((0,3), dtype=KERN_FLOAT)
        else:
            # Ensure MAX_MEASUREMENTS is valid int
            if self.MAX_MEASUREMENTS is None:
                self.MAX_MEASUREMENTS = 50
            # Prevent GPU Crash by capping measurements
            if len(measurements_rb) > self.MAX_MEASUREMENTS:
                measurements_rb = measurements_rb[:self.MAX_MEASUREMENTS]
            measurements_rb = ensure_contiguous(np.array(measurements_rb), KERN_FLOAT)
        print("Input Data given to Kernel")
        N_measurements = len(measurements_rb)

        # Reset Weights
        try:
            reset_weights_func = self.cuda_modules["resample"].get_function("reset_weights")
            reset_weights_func(self.memory.particles, block=(self.THREADS,1,1), grid=make_grid(self.N,self.THREADS))
            cuda.Context.synchronize()
            print_gpu_stats("reset_weights successful")
        except Exception as e:
            raise RuntimeError(f"reset_weights kernel failed: {e}")

        # Prediction
        try:
            prediction_func = self.cuda_modules["predict"].get_function("predict_from_model")
            prediction_func(
                self.memory.particles,
                np.float64(omega_angular),
                np.float64(v_linear),
                np.float64(np.sqrt(self.config.CONTROL_VARIANCE[0])),
                np.float64(np.sqrt(self.config.CONTROL_VARIANCE[1])),
                np.float64(dt),                    #Using calculated dt
                block=(self.THREADS,1,1),
                grid=make_grid(self.N,self.THREADS)
            )
            cuda.Context.synchronize()
            print_gpu_stats("predict_from_model successful")
        except Exception as e:
            raise RuntimeError(f"predict_from_model failed: {e}")

        # Update
        if N_measurements > 0:
            try:
                check_and_memcpy_htod(self.memory.measurements, measurements_rb, name="measurements")
                update_func = self.cuda_modules["update"].get_function("update")
                update_func(
                    self.memory.particles,
                    np.int32(self.N// self.THREADS),
                    self.memory.scratchpad,
                    np.int32(self.memory.scratchpad_block_size),
                    self.memory.measurements,
                    np.int32(self.N),
                    np.int32(N_measurements),
                    self.memory.cov,
                    np.float64(self.config.THRESHOLD),
                    np.float64(self.config.sensor.RANGE),
                    np.float64(self.config.sensor.FOV),
                    np.int32(self.config.MAX_LANDMARKS),
                    block=(self.THREADS,1,1),
                    grid=make_grid(self.N, self.THREADS)
                )
                cuda.Context.synchronize()
                print_gpu_stats("update kernel successful")
            except Exception as e:
                raise RuntimeError(f"update kernel or HtoD failed: {e}\n{traceback.format_exc()}")

        # Rescale & Pose Estimate
        try:
            rescale(self.cuda_modules, self.config, self.memory)
            estimated_pose = get_pose_estimate(self.cuda_modules, self.config, self.memory)
            cuda.Context.synchronize()
            print_gpu_stats("get_pose_estimate")
        except Exception as e:
            raise RuntimeError(f"get_pose_estimate failed: {e}")

        # Compute Neff & Resample
        try:
            weights_func = self.cuda_modules["weights_and_mean"].get_function("get_weights")
            weights_func(self.memory.particles, self.memory.weights, block=(self.THREADS,1,1), grid=make_grid(self.N,self.THREADS))
            cuda.Context.synchronize()
            check_and_memcpy_dtoh(self.weights, self.memory.weights, name="weights")
            neff = FlatParticle.neff(self.weights.astype(np.float64))
            if neff < 0.3*self.N:
                resample(self.cuda_modules, self.config, self.weights, self.memory, 0.5)
                cuda.Context.synchronize()
                print_gpu_stats("resample due to low Neff")
        except Exception as e:
            raise RuntimeError(f"weights_and_mean / resample failed: {e}")

        # Extract Map
        try:
            check_and_memcpy_dtoh(self.particles, self.memory.particles, name="particles")
            best_idx = int(np.argmax(FlatParticle.w(self.particles)))
            best_landmarks = FlatParticle.get_landmarks(self.particles, best_idx)
            best_covariances = FlatParticle.get_covariances(self.particles, best_idx)
            best_colors = FlatParticle.get_colors(self.particles, best_idx)
            
            print("Post_Processing Particels successful")
        except Exception as e:
            raise RuntimeError(f"postprocessing particles failed: {e}")

        final_map_data = []
        # Zip safely (stops at the shortest list to prevent index errors)
        limit = min(len(best_landmarks), len(best_colors))
        
        for i in range(limit):
            lm = best_landmarks[i]
            cov = best_covariances[i]
            color_val = best_colors[i]
            
            # Filter uninitialized landmarks
            if not np.all(cov == 0) and np.linalg.norm(lm[:2]) > 0.01:
                final_map_data.append({
                    "position": lm, 
                    "covariance": cov, 
                    "type": int(color_val)
                })
        print(final_map_data)
        print(estimated_pose)

        return estimated_pose, final_map_data

# ---------- ROS 2 Node ----------
class FastSLAMNode(Node):
    def __init__(self):
        super().__init__("fastslam_node")
        # Read the limit from the imported config file
        self.MAX_MEASUREMENTS = config.sensor.MAX_MEASUREMENTS
        self.get_logger().info("FastSLAM Node Initializing...")

        self.last_time = None      # Initialize variable to store previous time
        self.slam_core = FastSLAMCore(config)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/slam/pose", 1)
        self.cone_map_pub = self.create_publisher(ConeArray, "/slam/cone_map", 1)
        self.map_markers_pub = self.create_publisher(MarkerArray, "/slam/cone_map_markers", 1)
        # Subscribers
        self.odom_sub = Subscriber(self, Odometry, "/odom")
        self.cone_sub = Subscriber(self, ConeArray, "/cone_array")

        # Input topics Synchronizer
        self.ts = ApproximateTimeSynchronizer([self.odom_sub, self.cone_sub], queue_size=20, slop=0.1)  # slop=0.1  allows ~0.1s mismatch
        self.ts.registerCallback(self.slam_callback)
        self.get_logger().info("Subscribers synchronized. Waiting for data...")

    def slam_callback(self, odom_msg: Odometry, cone_array_msg: ConeArray):
        v_linear = odom_msg.twist.twist.linear.x
        omega_angular = odom_msg.twist.twist.angular.z
        rx = odom_msg.pose.pose.position.x
        ry = odom_msg.pose.pose.position.y
        rq = odom_msg.pose.pose.orientation
        r_yaw = quaternion_to_yaw(rq.x, rq.y, rq.z, rq.w)
         
        measurements_rb = self._process_cones(cone_array_msg, rx, ry, r_yaw)   # cone_types not being used yet in this code
        # Calculate DT
        current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return               # Skip the first frame so we have a valid interval next time
        dt = current_time - self.last_time
        self.last_time = current_time
        # Safety check for jumps or lag
        if dt <= 0.0 or dt > 0.5: 
            self.get_logger().warn(f"Abnormal DT detected: {dt:.4f}s. Resetting timer.")
            return

        # Skip update if stationary AND no valid measurements
        if v_linear == 0.0 and omega_angular == 0.0 and len(measurements_rb) == 0:
            return

        try:
            estimated_pose, final_map_data = self.slam_core.execute_step(v_linear, omega_angular, measurements_rb, dt)
        except Exception as e:
            self.get_logger().error(f"SLAM execution error: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        # Execution
        self._publish_transforms(odom_msg, estimated_pose)
        self._publish_pose_stamped(estimated_pose, odom_msg.header.stamp)
        self._publish_cone_map(final_map_data, odom_msg.header.stamp)
        self._publish_map_markers(final_map_data, odom_msg.header.stamp)
        
    def _process_cones(self, cone_array_msg: ConeArray, rx, ry, r_yaw):
        """
        Extracts cones. If frame_id is 'map', transforms them to 'base_link' 
        so SLAM can process them as range/bearing.
        """
        all_cones_rb = []   # store [range, bearing, cone_type] of all detected cones
        
        is_map_frame = (cone_array_msg.header.frame_id == "map")
        LIDAR_X_OFFSET = 0.52     # The offset of velodyne coordinate to base_link coordinate.
        cone_mapping = [
            (cone_array_msg.yellow_cones, Cone.YELLOW),
            (cone_array_msg.blue_cones, Cone.BLUE),
            (cone_array_msg.orange_cones, Cone.ORANGE),
            (cone_array_msg.large_orange_cones, Cone.LARGE_ORANGE),
            (cone_array_msg.unknown_cones, Cone.UNKNOWN),
        ]
        
        for cone_list, cone_type in cone_mapping:
            for cone in cone_list:
                cx, cy = cone.position.x, cone.position.y
                if is_map_frame:
                    # TRANSFORM: Global (Map) -> Local (Base Link)
                    dx = cx - rx
                    dy = cy - ry
                    # Rotate into robot frame
                    cos_v = math.cos(r_yaw)
                    sin_v = math.sin(r_yaw)
                    local_x = dx * cos_v + dy * sin_v
                    local_y = -dx * sin_v + dy * cos_v
                else:
                    # Frame is 'velodyne', so we add offset to get to 'base_link'
                    local_x = cx + LIDAR_X_OFFSET
                    local_y = cy

                r, beta = xy_to_range_bearing(local_x, local_y)

                #- FILTER 1: HARD DISTANCE CUTOFF ---
                # Anything further than 15m is too noisy -> Ignore it.
                if r > 15.0 or r<1.5:
                   continue
                all_cones_rb.append([r, beta,float(cone_type)])            
        # Convert to numpy for sorting
        measurements_rb = np.array(all_cones_rb, dtype=KERN_FLOAT)
        
        #- FILTER 2: KEEP ONLY CLOSEST 50 ---
        if len(measurements_rb) > 0:
            # Sort by distance (column 0)
            sorted_indices = np.argsort(measurements_rb[:, 0])
            measurements_rb = measurements_rb[sorted_indices]
            # Keep cones till allowed max limit and forget others
            if len(measurements_rb) > self.MAX_MEASUREMENTS:
                measurements_rb = measurements_rb[:self.MAX_MEASUREMENTS]    
        return measurements_rb

    def _publish_transforms(self, odom_msg: Odometry, estimated_pose: np.ndarray):
        x_map, y_map, theta_map = estimated_pose[0], estimated_pose[1], estimated_pose[2]
        
        odom_x = odom_msg.pose.pose.position.x
        odom_y = odom_msg.pose.pose.position.y
        odom_q = odom_msg.pose.pose.orientation
        odom_theta = quaternion_to_yaw(odom_q.x, odom_q.y, odom_q.z, odom_q.w)

        delta_theta = theta_map - odom_theta
        
        # Calculate the origin of the odom frame relative to the map frame
        t_x = x_map - (odom_x * math.cos(delta_theta) - odom_y * math.sin(delta_theta))
        t_y = y_map - (odom_x * math.sin(delta_theta) + odom_y * math.cos(delta_theta))
        
        q_corr_msg = yaw_to_quaternion(delta_theta)

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = t_x
        t.transform.translation.y = t_y
        t.transform.translation.z = 0.0
        t.transform.rotation = q_corr_msg
        self.tf_broadcaster.sendTransform(t)

    def _publish_pose_stamped(self, estimated_pose: np.ndarray, timestamp):
        x, y, theta = estimated_pose[0], estimated_pose[1], estimated_pose[2]
        q_msg = yaw_to_quaternion(theta)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "map" 
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = q_msg
        self.pose_pub.publish(pose_msg)
    def _publish_cone_map(self, final_map_data, stamp):
        msg = ConeArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"      # Check if required is map or odom

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

    def _publish_map_markers(self,final_map_data , stamp):
        marker_array = MarkerArray()
        # Use 'enumerate' to generate a temporary ID since we don't have a buffer ID anymore
        for i, lm in enumerate(final_map_data):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "cone_map_markers"
            marker.id = i # Simple index ID
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(lm["position"][0])
            marker.pose.position.y = float(lm["position"][1])
            marker.pose.position.z = 0.15
            
            color = get_cone_color(int(lm["type"]))
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5
            marker.color.a = color["a"]
            marker.color.r = color["r"]
            marker.color.g = color["g"]
            marker.color.b = color["b"]
            marker_array.markers.append(marker)
        
        # IMPORTANT: Delete markers that are no longer in the list
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, delete_marker)
        
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
