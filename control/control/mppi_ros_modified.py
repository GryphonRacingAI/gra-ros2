#!/usr/bin/env python3
"""MPPI path-following controller.

Inputs: /odom (vehicle state, odom frame), /path (from the pathfinder), /slam/cones (PointCloud2
obstacles, odom frame), /steer_angle (measured steering angle).
/path may be in the odom frame (global) or body-relative with the car at the origin and +x forward
(frame_id 'velodyne' or 'base_link', as the local-frame track_pathfinder publishes). The controller
follows the frame_id of each path: for a local path it plans from (0, 0, 0) and moves the cones into
the car frame; otherwise it plans from the odom pose.
Output: /ackermann_cmd (speed request + steering angle request), as the ADS-DV VCU takes them.

Rollout model: kinematic bicycle with the steering actuator (first-order lag + rate limit) and
the VCU's acceleration limits. Cost: distance to the path (nearest point, not time-indexed),
heading error, tracking a curvature- and braking-limited speed profile, staying under the
lateral grip limit, smooth controls, distance to cones.
"""
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node

from control.path_processor import PathProcessor, PathProcessorConfig
from fsai_common.geometry import to_local, to_world, yaw_from_quaternion

# ROS Messages
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

import sensor_msgs_py.point_cloud2 as pc2


def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)


def normalize_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))


def resample_path(points, spacing):
    """Resample a polyline at a fixed arc-length spacing. Returns (points, cumulative length)."""
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-6:
        return points[:1], np.zeros(1)
    s_new = np.arange(0.0, s[-1] + 1e-9, spacing)
    return np.column_stack([np.interp(s_new, s, points[:, 0]), np.interp(s_new, s, points[:, 1])]), s_new


def rollout(X0, U, dt, p):
    """Kinematic bicycle with steering actuator, vectorised over K rollouts.

    X0: (5,) [x, y, theta, v, delta]   U: (K, H, 2) [accel, steering command]
    Returns X: (K, H+1, 5)
    """
    K, H, _ = U.shape
    X = np.empty((K, H + 1, 5))
    X[:, 0, :] = X0
    max_dsteer = p['steer_rate'] * dt
    for h in range(H):
        th, v, de = X[:, h, 2], X[:, h, 3], X[:, h, 4]
        a = clamp(U[:, h, 0], -p['max_decel'], p['max_accel'])
        cmd = clamp(U[:, h, 1], -p['max_steer'], p['max_steer'])
        de_next = de + clamp((cmd - de) * dt / p['steer_tau'], -max_dsteer, max_dsteer)
        v_next = clamp(v + a * dt, p['v_creep'], p['max_vel'])
        v_mid = 0.5 * (v + v_next)
        th_mid = th + 0.5 * v_mid * np.tan(de) / p['wheelbase'] * dt
        X[:, h + 1, 0] = X[:, h, 0] + v_mid * np.cos(th_mid) * dt
        X[:, h + 1, 1] = X[:, h, 1] + v_mid * np.sin(th_mid) * dt
        X[:, h + 1, 2] = th + v_mid * np.tan(de) / p['wheelbase'] * dt
        X[:, h + 1, 3] = v_next
        X[:, h + 1, 4] = de_next
    return X


def rollout_cost(P, U, steer0, ref, ref_heading, ref_v, obstacles, c, debug=False):
    """Cost of every sampled rollout. No ROS: `c` is anything with the weights as attributes (the node,
    or a SimpleNamespace in a test).

    P (K, H, 5): rollout states [x, y, yaw, v, steer] of the rear axle after each step; U (K, H, 2): the
    controls [accel, steer]; steer0: steering angle now; ref (M, 2), ref_heading (M,), ref_v (M,): the
    reference path, its heading and speed; obstacles (N, 3): cones [x, y, radius] in the plan frame.
    Returns (cost (K,), parts): parts is the per-term breakdown if debug, else None.
    """
    K = P.shape[0]
    hd = np.stack([np.cos(P[:, :, 2]), np.sin(P[:, :, 2])], axis=-1)   # (K, H, 2)
    centre = P[:, :, :2] + c.rear_to_base * hd                          # centre of the car
    diff = centre[:, :, None, :] - ref[None, None, :, :]             # (K, H, M, 2)
    dist2 = np.einsum('khmi,khmi->khm', diff, diff)
    j = np.argmin(dist2, axis=2)                                    # nearest path point
    d2 = np.take_along_axis(dist2, j[..., None], axis=2)[..., 0]
    cost = c.w_path * d2.sum(axis=1)
    parts = {'path': cost.copy()} if debug else None
    h_err = normalize_angle(P[:, :, 2] - ref_heading[j])
    cost += c.w_heading * (h_err ** 2).sum(axis=1)
    # progress along the path over the horizon; without it, stopping in a tight corner can be
    # cheaper than driving through it
    cost -= c.w_progress * j[:, -1] * c.path_spacing
    dv = P[:, :, 3] - ref_v[j]
    cost += (np.where(dv > 0.0, c.w_overspeed, c.w_speed) * dv ** 2).sum(axis=1)
    a_lat = P[:, :, 3] ** 2 * np.tan(P[:, :, 4]) / c.p['wheelbase']
    cost += c.w_grip * (np.clip(np.abs(a_lat) - c.a_lat_max * 1.2, 0.0, None) ** 2).sum(axis=1)
    dsteer = np.diff(np.concatenate([np.full((K, 1), steer0), U[:, :, 1]], axis=1), axis=1)
    cost += c.w_steer_rate * (dsteer ** 2).sum(axis=1)
    cost += c.w_accel * (U[:, :, 0] ** 2).sum(axis=1)
    if parts is not None:
        parts['progress'] = -c.w_progress * j[:, -1] * c.path_spacing
        parts['speed'] = (np.where(dv > 0.0, c.w_overspeed, c.w_speed) * dv ** 2).sum(axis=1)
        parts['obst'] = np.zeros(K)

    if obstacles.shape[0] > 0:
        # all footprint circles at once in float32: a loop over circles in float64 took ~110 ms
        # with 50 cones and dropped the 20 Hz control loop to ~10 Hz
        # cones within obstacle_window m of the previous plan (the samples stay close to it; the
        # map has 50+ cones in reach later in the run and the cost grows with K x H x circles x cones)
        nominal = P[0, ::3, :2]                                               # sample 0 = previous plan
        d_nom = np.min(np.linalg.norm(obstacles[None, :, :2] - nominal[:, None, :], axis=-1), axis=0)
        obs = obstacles[d_nom < c.obstacle_window].astype(np.float32)
        if obs.shape[0] > 0:
            cpos = (P[:, :, None, :2].astype(np.float32)
                    + c.circle_offsets.astype(np.float32)[None, None, :, None] * hd.astype(np.float32)[:, :, None, :])
            dx = cpos[..., None, 0] - obs[:, 0]                                      # (K, H, C, N)
            dy = cpos[..., None, 1] - obs[:, 1]
            d_obs = np.sqrt(dx * dx + dy * dy) - obs[:, 2] - c.circle_radii.astype(np.float32)[None, None, :, None]
            d_pen = np.maximum(c.obstacle_margin - d_obs, 0.0)
            c_obs = np.minimum(c.w_obstacle * (d_pen * d_pen).sum(axis=(1, 3)), 1e4).sum(axis=1)
            cost += c_obs
            if parts is not None:
                parts['obst'] = c_obs
            if c.w_collision > 0.0:
                cost += c.w_collision * (d_obs < 0.0).sum(axis=(1, 2, 3))
    return cost, parts


class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_controller')

        self.path_topic = self.declare_parameter('path_topic', '/path').value
        # Local-frame mode: a /path whose frame_id is path_frame / base_link / velodyne (or has no frame
        # id) is body-relative, car at the origin. Set use_local_frame:=false to force odom-frame paths.
        self.use_local_frame = self.declare_parameter('use_local_frame', True).value
        self.expected_path_frame = self.declare_parameter('path_frame', 'velodyne').value

        # MPPI Core Parameters
        self.dt = self.declare_parameter('dt', 0.05).value
        self.horizon = self.declare_parameter('horizon', 30).value
        self.num_rollouts = self.declare_parameter('num_rollouts', 400).value
        self.lambda_ = self.declare_parameter('lambda', 2.0).value
        self.lambda_spread = self.declare_parameter('lambda_spread', 0.15).value  # temperature >= this x (median - best cost)
        self.sigma_u = np.array(self.declare_parameter('sigma_u', [1.5, 0.06]).value)
        self.alpha = self.declare_parameter('alpha', 0.6).value  # time correlation of the noise

        # Cost Weights
        self.w_path = self.declare_parameter('w_path', 20.0).value          # (distance to path)^2
        self.w_heading = self.declare_parameter('w_heading', 4.0).value     # (heading error)^2
        self.w_speed = self.declare_parameter('w_speed', 1.0).value         # (v - v_ref)^2 below v_ref
        self.w_overspeed = self.declare_parameter('w_overspeed', 5.0).value  # (v - v_ref)^2 above v_ref
        self.w_progress = self.declare_parameter('w_progress', 10.0).value  # reward per metre along the path
        self.w_grip = self.declare_parameter('w_grip', 20.0).value           # (a_lat over limit)^2
        self.w_steer_rate = self.declare_parameter('w_steer_rate', 150.0).value  # (d steer cmd)^2
        self.w_accel = self.declare_parameter('w_accel', 0.05).value        # accel^2
        self.w_obstacle = self.declare_parameter('w_obstacle', 300.0).value

        # Vehicle (ADS-DV: VCU torque limits, steering actuator; see ads_dv_vcu_sim.cpp)
        self.p = {
            'max_accel': self.declare_parameter('max_accel', 4.0).value,
            'max_decel': self.declare_parameter('max_decel', 4.0).value,
            'max_vel': self.declare_parameter('max_vel', 15.0).value,
            # never plan to stop while there is a path: a stopped car can't steer, so "stand still"
            # became a dead end in tight corners. Stopping is mission_control's job (brakes).
            'v_creep': self.declare_parameter('v_creep', 1.5).value,
            'max_steer': self.declare_parameter('max_steer', 0.38).value,
            'steer_rate': self.declare_parameter('steer_rate', 1.0).value,
            'steer_tau': self.declare_parameter('steer_tau', 0.15).value,
            'wheelbase': self.declare_parameter('wheelbase', 1.534).value,
        }

        # Path Following
        self.path_spacing = self.declare_parameter('path_spacing', 0.5).value
        self.path_smoothing = self.declare_parameter('path_smoothing', 1.5).value  # m, half window
        # Car footprint for the cone cost. The model point is the REAR AXLE (kinematic bicycle); base_link
        # (centre of the car, where /odom is) is rear_to_base ahead of it and the path frame (velodyne) is
        # base_to_path_frame further ahead. Cones are checked with circles along the car (offsets from the
        # rear axle, covering the body from the tail to the nose), with the radius that covers the body or the wheels there.
        self.rear_to_base = self.declare_parameter('rear_to_base', 0.767).value
        self.base_to_path_frame = self.declare_parameter('base_to_path_frame', 0.52).value
        self.circle_offsets = np.array(self.declare_parameter('circle_offsets', [-0.5, 0.0, 0.767, 1.534, 1.95]).value, dtype=float)
        self.circle_radii = np.array(self.declare_parameter('circle_radii', [0.35, 0.67, 0.40, 0.58, 0.30]).value, dtype=float)
        self.obstacle_margin = self.declare_parameter('obstacle_margin', 0.3).value  # m of clearance wanted beyond touching
        self.w_collision = self.declare_parameter('w_collision', 0.0).value        # per step, per circle, touching a cone
        self.cone_radius = self.declare_parameter('cone_radius', 0.12).value
        self.obstacle_window = self.declare_parameter('obstacle_window', 5.0).value  # m around the previous plan
        self.debug_costs = self.declare_parameter('debug_costs', False).value  # log the cost terms of the best sample every 1 s
        self.speed_lead = self.declare_parameter('speed_lead', 0.25).value  # s ahead for the speed request
        self.path_timeout = self.declare_parameter('path_timeout', 1.0).value  # s without a new path -> stop
        self.path_extension = self.declare_parameter('path_extension', 0.0).value  # m, straight events only

        # Speed Profile
        self.a_lat_max = self.declare_parameter('a_lat_max', 3.5).value  # planned; tyres give ~13
        self.v_max_straight = self.declare_parameter('v_max_straight', 10.0).value
        self.v_min = self.declare_parameter('v_min', 2.0).value
        self.a_brake_plan = self.declare_parameter('a_brake_plan', 3.0).value
        # planned acceleration out of slow sections: accelerating hard while still steering out of a
        # hairpin pushed the car wide into the exit cones (tyre grip is shared by both directions)
        self.a_accel_plan = self.declare_parameter('a_accel_plan', 2.0).value  # 0 = no limit

        self.path_processor = PathProcessor(PathProcessorConfig(
            a_lat_max=self.a_lat_max,
            v_max_straight=self.v_max_straight,
            v_min=self.v_min,
        ))

        self.get_logger().info(f"[MPPI] Using path topic: {self.path_topic}")

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.sub_cones = self.create_subscription(PointCloud2, '/slam/cones', self.cone_callback, 10)
        # Cones seen by the lidar right now, colour or not (lidar_cone_obstacles.py), added to the SLAM
        # cones: the apex cones of a hairpin are beside the car, outside the camera's view, and often
        # missing from the map. Empty topic name: SLAM cones only.
        self.lidar_obstacles = np.empty((0, 3))
        self.lidar_obstacles_time = None
        self.lidar_obstacles_max_age = self.declare_parameter('lidar_obstacles_max_age', 0.3).value
        lidar_topic = self.declare_parameter('lidar_obstacles_topic', '/lidar_cones').value
        if lidar_topic:
            self.create_subscription(PointCloud2, lidar_topic, self.lidar_cone_callback, 1)
        self.sub_steer = self.create_subscription(Float64, '/steer_angle', self.steer_callback, 10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pub_ackermann = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        self.pub_viz_path = self.create_publisher(Path, '/viz/mppi_path', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.vehicle_state = None   # odom-frame pose + speed
        self.path_is_local = False
        self.path_frame_id = 'odom'
        self.obstacles_frame = 'odom'
        self.steer_measured = None
        self.path_arr = None
        self.path_heading = None
        self.path_v_ref = None
        self.last_path_time = None
        self.obstacles = np.empty((0, 3))

        self.u0 = np.zeros((self.horizon, 2))
        self._loop_stats = []
        self._dbg_n = 0
        self.rng = np.random.default_rng()

        self.get_logger().info(
            f"MPPI Controller Initialized (local paths in '{self.expected_path_frame}' "
            f"{'accepted' if self.use_local_frame else 'disabled'})")

    def odom_callback(self, msg):
        """ Update vehicle state from Odometry """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q)
        self.vehicle_state = np.array([p.x, p.y, yaw, msg.twist.twist.linear.x], dtype=float)

    def steer_callback(self, msg):
        self.steer_measured = float(msg.data)

    def path_callback(self, msg):
        """Receive a path, resample it evenly and build the reference heading and speed."""
        if len(msg.poses) < 2:
            return
        frame = msg.header.frame_id or ''
        self.path_frame_id = frame or ('odom' if not self.use_local_frame else self.expected_path_frame)
        self.path_is_local = self.use_local_frame and frame in (self.expected_path_frame, 'base_link', 'velodyne', '')
        raw = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if self.path_extension > 0.0:
            # straight event: continue the path past what the planner can see, so the horizon
            # (1.5 s) doesn't cap the speed at path length / 1.5 s
            d = raw[-1] - raw[-2]
            raw = np.vstack([raw, raw[-1] + d / (np.linalg.norm(d) + 1e-9) * self.path_extension])
        # The planner's waypoints are gate midpoints that jitter by tens of cm between updates;
        # differentiating them for curvature gave spurious tight "corners" and a speed profile that
        # collapsed (4 m/s on a straight). Smooth the geometry first (moving average over
        # +-path_smoothing m, start point kept).
        dense, _ = resample_path(raw, self.path_spacing)
        k = int(self.path_smoothing / self.path_spacing)
        if k > 0 and len(dense) > 2 * k + 1:
            pad = np.vstack([np.repeat(dense[:1], k, axis=0), dense, np.repeat(dense[-1:], k, axis=0)])
            kernel = np.ones(2 * k + 1) / (2 * k + 1)
            smooth = np.column_stack([np.convolve(pad[:, i], kernel, mode='valid') for i in (0, 1)])
            smooth[0] = dense[0]
            dense = smooth
        result = self.path_processor(dense if len(dense) >= 2 else raw)
        pts, s = resample_path(result.points, self.path_spacing)
        if len(pts) < 2:
            return
        # heading and curvature-limited speed from the team's PathProcessor, at the new spacing
        s_orig = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(result.points, axis=0), axis=1))])
        heading = np.interp(s, s_orig, np.unwrap(result.heading))
        # Speed from curvature over a +-3 m baseline (circle through three points): differentiating
        # point to point (PathProcessor) turns 10 cm of jitter into a "tight corner".
        b = max(1, int(3.0 / self.path_spacing))
        kappa = np.zeros(len(pts))
        for i in range(len(pts)):
            a_, c_ = pts[max(0, i - b)], pts[min(len(pts) - 1, i + b)]
            p_ = pts[i]
            ab, bc, ac = np.linalg.norm(p_ - a_), np.linalg.norm(c_ - p_), np.linalg.norm(c_ - a_)
            area2 = abs((p_[0] - a_[0]) * (c_[1] - a_[1]) - (p_[1] - a_[1]) * (c_[0] - a_[0]))
            kappa[i] = 2.0 * area2 / (ab * bc * ac) if ab * bc * ac > 1e-6 else 0.0
        v_ref = np.clip(np.sqrt(self.a_lat_max / (kappa + 1e-3)), self.v_min, self.v_max_straight)
        # Drive within sight: be able to slow to v_min by the end of what the planner can see (the
        # path is only a few metres long in an unmapped corner on lap 1). Then brake in time for
        # slow sections ahead (backward pass).
        v_ref[-1] = min(v_ref[-1], self.v_min)
        for i in range(len(v_ref) - 2, -1, -1):
            v_ref[i] = min(v_ref[i], math.sqrt(v_ref[i + 1] ** 2 + 2.0 * self.a_brake_plan * (s[i + 1] - s[i])))
        if self.a_accel_plan > 0.0:  # forward pass: gradual acceleration out of slow sections
            for i in range(1, len(v_ref)):
                v_ref[i] = min(v_ref[i], math.sqrt(v_ref[i - 1] ** 2 + 2.0 * self.a_accel_plan * (s[i] - s[i - 1])))
        self.path_arr, self.path_heading, self.path_v_ref = pts, heading, v_ref
        self.last_path_time = self.get_clock().now()

    def cone_callback(self, msg):
        """ Parse PointCloud2 cones into obstacle array """
        # read_points() returns a structured array on Jazzy; read_points_numpy gives a plain (N, 2)
        points_np = pc2.read_points_numpy(msg, field_names=("x", "y"), skip_nans=True).reshape(-1, 2)
        if points_np.shape[0] == 0:
            self.obstacles = np.empty((0, 3))
            return
        r = self.cone_radius * np.ones((points_np.shape[0], 1))
        self.obstacles = np.hstack([points_np, r])
        self.obstacles_frame = msg.header.frame_id or 'odom'

    def lidar_cone_callback(self, msg):
        pts = pc2.read_points_numpy(msg, field_names=("x", "y"), skip_nans=True).reshape(-1, 2)
        if (msg.header.frame_id or 'odom') != 'odom':
            return  # only odom-frame clouds are merged with the SLAM cones
        self.lidar_obstacles = np.hstack([pts, self.cone_radius * np.ones((len(pts), 1))])
        self.lidar_obstacles_time = self.get_clock().now()

    def plan_frame_ahead(self):
        """How far the origin of a local plan frame is ahead of base_link."""
        return self.base_to_path_frame if self.path_frame_id == 'velodyne' else 0.0

    def obstacles_in(self, local):
        """Obstacles in the frame the plan is made in. A local plan needs cones relative to the car."""
        obs, frame = self.obstacles, self.obstacles_frame
        if (self.lidar_obstacles.shape[0] > 0 and self.lidar_obstacles_time is not None
                and (obs.shape[0] == 0 or frame == 'odom')
                and (self.get_clock().now() - self.lidar_obstacles_time).nanoseconds * 1e-9 < self.lidar_obstacles_max_age):
            # the lidar cones are in odom; without SLAM cones they are the whole obstacle set
            obs = np.vstack([obs, self.lidar_obstacles]) if obs.shape[0] > 0 else self.lidar_obstacles
            frame = 'odom'
        cones_local = frame in (self.expected_path_frame, 'base_link', 'velodyne')
        if obs.shape[0] == 0 or local == cones_local:
            return obs
        x, y, yaw, _ = self.vehicle_state
        out = obs.copy()
        if local:   # odom -> the car's base_link, then moved to the origin of the path frame
            out[:, :2] = to_local(obs[:, :2], x, y, yaw) - [self.plan_frame_ahead(), 0.0]
        else:       # car -> odom
            out[:, :2] = to_world(obs[:, :2], x, y, yaw)
        return out

    def control_loop(self):
        """ Main MPPI Iteration (Called at 1/dt Hz) """
        _t_loop = time.perf_counter()

        # Don't sleep here: with use_sim_time the clock only advances when this (single-threaded)
        # executor processes /clock, so sleep_for() blocked the node forever.
        if self.vehicle_state is None:
            self.get_logger().info("Waiting for vehicle state", throttle_duration_sec=3.0)
            return
        if self.path_arr is None:
            self.get_logger().info("Waiting for path", throttle_duration_sec=3.0)
            return

        local = self.path_is_local
        _, _, _, v = self.vehicle_state
        # model point = rear axle (see rear_to_base)
        if local:
            x, y, yaw = -(self.rear_to_base + self.plan_frame_ahead()), 0.0, 0.0
        else:
            bx, by, yaw = self.vehicle_state[:3]
            x, y = bx - self.rear_to_base * math.cos(yaw), by - self.rear_to_base * math.sin(yaw)
        # the path is followed by the centre of the car
        centre0 = np.array([x + self.rear_to_base * math.cos(yaw), y + self.rear_to_base * math.sin(yaw)])
        obstacles = self.obstacles_in(local)

        # No new path for a while (the planner sees no gates): keep following the last one, which
        # usually brings new cones into view, but stop at its end instead of circling there with
        # the creep speed (acceleration: the last gate before the finish line).
        stale = (self.get_clock().now() - self.last_path_time).nanoseconds * 1e-9 > self.path_timeout
        # only for straight events (path_extension > 0): elsewhere a short gap in the planner's output near
        # the end of the last path must not freeze the car for good (it keeps creeping and sees new cones)
        if self.path_extension > 0.0 and stale and np.linalg.norm(self.path_arr[-1] - centre0) < 2.0:
            self.get_logger().warn("reached the end of the last /path: stopping", throttle_duration_sec=2.0)
            self.pub_ackermann.publish(AckermannDrive(speed=0.0, steering_angle=0.0))
            self.u0[:] = 0.0
            return
        steer = self.steer_measured if self.steer_measured is not None else float(self.u0[0, 1])
        X0 = np.array([x, y, yaw, max(v, 0.0), steer])

        # path points from the closest one onwards, enough to cover the horizon
        d_car = np.linalg.norm(self.path_arr - centre0, axis=1)
        i0 = int(np.argmin(d_car))
        reach = (max(v, self.v_max_straight) * self.dt * self.horizon) / self.path_spacing + 10
        i1 = min(len(self.path_arr), i0 + int(reach))
        ref = self.path_arr[i0:i1]
        if len(ref) < 2:
            ref = self.path_arr[-2:]
            i0 = len(self.path_arr) - 2
            i1 = len(self.path_arr)
        ref_heading = self.path_heading[i0:i1]
        ref_v = self.path_v_ref[i0:i1].copy()
        if i1 == len(self.path_arr):
            # the path is only as long as the planner can see: don't treat its end as a stop line
            ref_v[-max(1, len(ref_v) // 3):] = np.maximum(ref_v[-max(1, len(ref_v) // 3):], self.v_min)

        # ---- sample controls (time-correlated noise) and roll out
        K, H = self.num_rollouts, self.horizon
        noise = self.rng.standard_normal((K, H, 2)) * self.sigma_u
        # Steering noise within the grip budget at the current speed: with a fixed sigma, fast
        # rollouts were penalised for sampled steering (v^2 tan(d) / L over the grip limit) more
        # often than slow ones, which biased the plan to lower speeds (plateau at ~13 of 18 m/s).
        noise[:, :, 1] *= min(1.0, self.a_lat_max * self.p['wheelbase'] / (max(v, 1.0) ** 2 * self.sigma_u[1]))
        for h in range(1, H):
            noise[:, h, :] = self.alpha * noise[:, h - 1, :] + math.sqrt(1 - self.alpha ** 2) * noise[:, h, :]
        noise[0] = 0.0  # keep the current plan as one candidate
        U = self.u0[None, :, :] + noise
        U[:, :, 1] = clamp(U[:, :, 1], -self.p['max_steer'], self.p['max_steer'])
        X = rollout(X0, U, self.dt, self.p)
        P = X[:, 1:, :]  # (K, H, 5)

        # ---- cost
        cost, parts = rollout_cost(P, U, steer, ref, ref_heading, ref_v, obstacles, self, self.debug_costs)
        if parts is not None:
            self._dbg_n += 1
            if self._dbg_n % 20 == 0:
                b = int(np.argmin(cost))
                self.get_logger().info(
                    f"costs(best) v={v:.1f} plan_v_end={P[b, -1, 3]:.1f} ref_v0={ref_v[0]:.1f} path_m={len(self.path_arr) * self.path_spacing:.1f} "
                    f"ref_m={len(ref) * self.path_spacing:.1f} total={cost[b]:.0f} " + ' '.join(f"{k}={v_[b]:.0f}" for k, v_ in parts.items())
                    + f" | fastest sample: v_end={P[:, -1, 3].max():.1f} total={cost[int(np.argmax(P[:, -1, 3]))]:.0f} "
                    + ' '.join(f"{k}={v_[int(np.argmax(P[:, -1, 3]))]:.0f}" for k, v_ in parts.items()))
        # ---- MPPI update
        # Adaptive temperature: when the costs of the samples are far apart (a cone in the way costs
        # hundreds), a fixed lambda gives all the weight to one sample and the steering jitters.
        lam = max(self.lambda_, self.lambda_spread * (float(np.median(cost)) - float(cost.min())))
        w = np.exp(-(cost - cost.min()) / lam)
        w /= w.sum() + 1e-12
        self.u0 = self.u0 + np.einsum('k,khi->hi', w, noise)
        self.u0[:, 0] = clamp(self.u0[:, 0], -self.p['max_decel'], self.p['max_accel'])
        self.u0[:, 1] = clamp(self.u0[:, 1], -self.p['max_steer'], self.p['max_steer'])
        # Smooth the steering sequence (moving average over 5 steps). Through the actuator lag the
        # first command barely changes the predicted motion, so the weighted average was noisy
        # and the car weaved.
        pad = np.concatenate([np.full(2, self.u0[0, 1]), self.u0[:, 1], np.full(2, self.u0[-1, 1])])
        self.u0[:, 1] = np.convolve(pad, np.ones(5) / 5.0, mode='valid')

        # ---- command: steering request now, speed request = planned speed speed_lead s ahead
        plan = rollout(X0, self.u0[None], self.dt, self.p)[0]
        lead = min(H, max(1, int(round(self.speed_lead / self.dt))))
        speed_cmd = float(max(plan[lead, 3], 0.0))  # never negative: the car reads that as "finished"
        steer_cmd = float(self.u0[0, 1])

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steer_cmd
        drive_msg.drive.speed = speed_cmd
        drive_msg.drive.acceleration = float(self.u0[0, 0])
        self.pub_drive.publish(drive_msg)

        ack_msg = AckermannDrive()
        ack_msg.steering_angle = steer_cmd
        ack_msg.speed = speed_cmd
        ack_msg.acceleration = float(self.u0[0, 0])
        self.pub_ackermann.publish(ack_msg)

        self.publish_viz_path(plan[:, :2], self.path_frame_id if local else 'odom')

        # timing: report the mean and worst loop time every 200 iterations (10 s); the loop must stay well
        # under dt, otherwise the timer runs back to back and starves perception/SLAM of CPU
        self._loop_stats.append((time.perf_counter() - _t_loop) * 1000.0)
        if len(self._loop_stats) >= 200:
            self.get_logger().info(f"loop time: mean {np.mean(self._loop_stats):.1f} ms, max {np.max(self._loop_stats):.1f} ms (dt {self.dt * 1000:.0f} ms)")
            self._loop_stats = []
        # Shift controls for receding horizon
        self.u0 = np.vstack([self.u0[1:], self.u0[-1:]])

    def publish_viz_path(self, path_pts, frame="odom"):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        for pt in path_pts:
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            msg.poses.append(pose)
        self.pub_viz_path.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPPIController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
