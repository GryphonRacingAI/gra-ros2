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

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node

from control.path_processor import PathProcessor, PathProcessorConfig

# ROS Messages
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

import sensor_msgs_py.point_cloud2 as pc2


def euler_from_quaternion(quat):
    x, y, z, w = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# Helper Functions
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
        self.sigma_u = np.array(self.declare_parameter('sigma_u', [1.5, 0.12]).value)
        self.alpha = self.declare_parameter('alpha', 0.6).value  # time correlation of the noise

        # Cost Weights
        self.w_path = self.declare_parameter('w_path', 20.0).value          # (distance to path)^2
        self.w_heading = self.declare_parameter('w_heading', 4.0).value     # (heading error)^2
        self.w_speed = self.declare_parameter('w_speed', 1.0).value         # (v - v_ref)^2 below v_ref
        self.w_overspeed = self.declare_parameter('w_overspeed', 5.0).value  # (v - v_ref)^2 above v_ref
        self.w_progress = self.declare_parameter('w_progress', 10.0).value  # reward per metre along the path
        self.w_grip = self.declare_parameter('w_grip', 5.0).value           # (a_lat over limit)^2
        self.w_steer_rate = self.declare_parameter('w_steer_rate', 50.0).value  # (d steer cmd)^2
        self.w_accel = self.declare_parameter('w_accel', 0.05).value        # accel^2
        self.w_obstacle = self.declare_parameter('w_obstacle', 150.0).value

        # Vehicle (ADS-DV: VCU torque limits, steering actuator; see ads_dv_vcu_sim.cpp)
        self.p = {
            'max_accel': self.declare_parameter('max_accel', 4.0).value,
            'max_decel': self.declare_parameter('max_decel', 4.0).value,
            'max_vel': self.declare_parameter('max_vel', 15.0).value,
            # never plan to stop while there is a path: a stopped car can't steer, so "stand still"
            # became a dead end in tight corners. Stopping is mission_control's job (brakes).
            'v_creep': self.declare_parameter('v_creep', 1.5).value,
            'max_steer': self.declare_parameter('max_steer', 0.4242).value,
            'steer_rate': self.declare_parameter('steer_rate', 1.0).value,
            'steer_tau': self.declare_parameter('steer_tau', 0.15).value,
            'wheelbase': self.declare_parameter('wheelbase', 1.534).value,
        }

        # Path Following
        self.path_spacing = self.declare_parameter('path_spacing', 0.5).value
        self.safety_distance = self.declare_parameter('safety_distance', 0.6).value
        self.cone_radius = self.declare_parameter('cone_radius', 0.2).value
        self.speed_lead = self.declare_parameter('speed_lead', 0.25).value  # s ahead for the speed request

        # Speed Profile
        self.a_lat_max = self.declare_parameter('a_lat_max', 5.0).value  # planned; tyres give ~13
        self.v_max_straight = self.declare_parameter('v_max_straight', 10.0).value
        self.v_min = self.declare_parameter('v_min', 2.0).value
        self.a_brake_plan = self.declare_parameter('a_brake_plan', 3.0).value

        self.path_processor = PathProcessor(PathProcessorConfig(
            a_lat_max=self.a_lat_max,
            v_max_straight=self.v_max_straight,
            v_min=self.v_min,
        ))

        self.get_logger().info(f"[MPPI] Using path topic: {self.path_topic}")

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.sub_cones = self.create_subscription(PointCloud2, '/slam/cones', self.cone_callback, 10)
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
        self.obstacles = np.empty((0, 3))

        self.u0 = np.zeros((self.horizon, 2))
        self.rng = np.random.default_rng()

        self.get_logger().info(
            f"MPPI Controller Initialized (local paths in '{self.expected_path_frame}' "
            f"{'accepted' if self.use_local_frame else 'disabled'})")

    def odom_callback(self, msg):
        """ Update vehicle state from Odometry """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
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
        result = self.path_processor(raw)
        pts, s = resample_path(result.points, self.path_spacing)
        if len(pts) < 2:
            return
        # heading and curvature-limited speed from the team's PathProcessor, at the new spacing
        s_orig = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(result.points, axis=0), axis=1))])
        heading = np.interp(s, s_orig, np.unwrap(result.heading))
        v_ref = np.interp(s, s_orig, result.speed_ref)
        # brake in time for slow sections ahead (backward pass)
        for i in range(len(v_ref) - 2, -1, -1):
            v_ref[i] = min(v_ref[i], math.sqrt(v_ref[i + 1] ** 2 + 2.0 * self.a_brake_plan * (s[i + 1] - s[i])))
        self.path_arr, self.path_heading, self.path_v_ref = pts, heading, v_ref

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

    def obstacles_in(self, local):
        """Obstacles in the frame the plan is made in. A local plan needs cones relative to the car."""
        obs = self.obstacles
        body_frames = (self.expected_path_frame, 'base_link', 'velodyne')
        cones_local = self.obstacles_frame in body_frames
        if obs.shape[0] == 0 or local == cones_local:
            return obs
        x, y, yaw, _ = self.vehicle_state
        c, s = math.cos(yaw), math.sin(yaw)
        out = obs.copy()
        if local:   # odom -> car
            dx, dy = obs[:, 0] - x, obs[:, 1] - y
            out[:, 0], out[:, 1] = c * dx + s * dy, -s * dx + c * dy
        else:       # car -> odom
            out[:, 0], out[:, 1] = x + c * obs[:, 0] - s * obs[:, 1], y + s * obs[:, 0] + c * obs[:, 1]
        return out

    def control_loop(self):
        """ Main MPPI Iteration (Called at 1/dt Hz) """

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
        x, y, yaw = (0.0, 0.0, 0.0) if local else self.vehicle_state[:3]
        obstacles = self.obstacles_in(local)
        steer = self.steer_measured if self.steer_measured is not None else float(self.u0[0, 1])
        X0 = np.array([x, y, yaw, max(v, 0.0), steer])

        # path points from the closest one onwards, enough to cover the horizon
        d_car = np.linalg.norm(self.path_arr - X0[:2], axis=1)
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
        for h in range(1, H):
            noise[:, h, :] = self.alpha * noise[:, h - 1, :] + math.sqrt(1 - self.alpha ** 2) * noise[:, h, :]
        noise[0] = 0.0  # keep the current plan as one candidate
        U = self.u0[None, :, :] + noise
        U[:, :, 1] = clamp(U[:, :, 1], -self.p['max_steer'], self.p['max_steer'])
        X = rollout(X0, U, self.dt, self.p)
        P = X[:, 1:, :]  # (K, H, 5)

        # ---- cost
        diff = P[:, :, None, :2] - ref[None, None, :, :]                 # (K, H, M, 2)
        dist2 = np.einsum('khmi,khmi->khm', diff, diff)
        j = np.argmin(dist2, axis=2)                                    # nearest path point
        d2 = np.take_along_axis(dist2, j[..., None], axis=2)[..., 0]
        cost = self.w_path * d2.sum(axis=1)
        h_err = normalize_angle(P[:, :, 2] - ref_heading[j])
        cost += self.w_heading * (h_err ** 2).sum(axis=1)
        # progress along the path over the horizon; without it, stopping in a tight corner can be
        # cheaper than driving through it
        cost -= self.w_progress * j[:, -1] * self.path_spacing
        dv = P[:, :, 3] - ref_v[j]
        cost += (np.where(dv > 0.0, self.w_overspeed, self.w_speed) * dv ** 2).sum(axis=1)
        a_lat = P[:, :, 3] ** 2 * np.tan(P[:, :, 4]) / self.p['wheelbase']
        cost += self.w_grip * (np.clip(np.abs(a_lat) - self.a_lat_max * 1.2, 0.0, None) ** 2).sum(axis=1)
        dsteer = np.diff(np.concatenate([np.full((K, 1), steer), U[:, :, 1]], axis=1), axis=1)
        cost += self.w_steer_rate * (dsteer ** 2).sum(axis=1)
        cost += self.w_accel * (U[:, :, 0] ** 2).sum(axis=1)

        if obstacles.shape[0] > 0:
            near = np.linalg.norm(obstacles[:, :2] - X0[:2], axis=1) < 20.0
            obs = obstacles[near]
            if obs.shape[0] > 0:
                d_obs = np.linalg.norm(P[:, :, None, :2] - obs[None, None, :, :2], axis=-1) - obs[None, None, :, 2]
                d_pen = np.clip(self.safety_distance - d_obs, 0.0, None)
                cost += np.clip(self.w_obstacle * np.sum(d_pen ** 2, axis=(1, 2)), 0.0, 1e4)

        # ---- MPPI update
        w = np.exp(-(cost - cost.min()) / self.lambda_)
        w /= w.sum() + 1e-12
        self.u0 = self.u0 + np.einsum('k,khi->hi', w, noise)
        self.u0[:, 0] = clamp(self.u0[:, 0], -self.p['max_decel'], self.p['max_accel'])
        self.u0[:, 1] = clamp(self.u0[:, 1], -self.p['max_steer'], self.p['max_steer'])

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
