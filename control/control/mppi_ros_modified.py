#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from control.path_processor import PathProcessor, PathProcessorConfig

# ROS Messages
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Header, String

import sensor_msgs_py.point_cloud2 as pc2
import math


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

def dynamics_vec(X, U, dt, max_accel, max_steer, min_vel, max_vel):
    """
    Vectorized dynamics:
    X: (K, H+1, 4)  [x, y, theta, v]
    U: (K, H, 2)    [accel, steer_angle]
    """
    K_curr, H_curr, _ = U.shape
    
    # Note: X is mutable and updated in place for next steps
    for h in range(H_curr):
        a = clamp(U[:, h, 0], -max_accel, max_accel)
        steer = clamp(U[:, h, 1], -max_steer, max_steer)
        
        theta = X[:, h, 2]
        v = X[:, h, 3]

        # Standard Kinematic Bicycle Model Update
        # dx = v * cos(theta) * dt
        # dy = v * sin(theta) * dt
        # dtheta = v * tan(steer) / L * dt  (Standard) 
        # OR dtheta = steer * dt (Simplified model from original script)
        
        # Using the Simplified Model from original script to preserve behavior:
        X[:, h+1, 0] = X[:, h, 0] + dt * v * np.cos(theta)
        X[:, h+1, 1] = X[:, h, 1] + dt * v * np.sin(theta)
        X[:, h+1, 2] = theta + dt * steer # Note: This treats steer as yaw rate
        X[:, h+1, 3] = clamp(v + dt * a, min_vel, max_vel)

    return X

class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_controller')

        self.path_topic = self.declare_parameter('path_topic', '/path').value

        # MPPI Core Parameters
        self.dt = self.declare_parameter('dt', 0.05).value
        self.horizon = self.declare_parameter('horizon', 12).value
        self.num_rollouts = self.declare_parameter('num_rollouts', 500).value
        self.lambda_ = self.declare_parameter('lambda', 2.0).value
        self.sigma_u_base = np.array(self.declare_parameter('sigma_u_base', [0.6, 0.15]).value)
        self.sigma_u_min = np.array(self.declare_parameter('sigma_u_min', [0.2, 0.05]).value)

        # Cost Weights
        self.w_path = self.declare_parameter('w_path', 40.0).value
        self.w_heading = self.declare_parameter('w_heading', 5.0).value
        self.w_speed = self.declare_parameter('w_speed', 2.0).value
        self.w_control = self.declare_parameter('w_control', 4.5).value
        self.w_terminal = self.declare_parameter('w_terminal', 1.0).value
        self.w_obstacle = self.declare_parameter('w_obstacle', 150.0).value

        # Vehicle Limits
        self.max_accel = self.declare_parameter('max_accel', 3.0).value
        self.min_vel = self.declare_parameter('min_vel', -1.0).value
        self.max_vel = self.declare_parameter('max_vel', 8.0).value
        self.max_steer = self.declare_parameter('max_steer', np.pi/6).value

        # Vehicle Geometry
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value

        # Path Following
        self.search_window = self.declare_parameter('search_window', 50).value
        self.safety_distance = self.declare_parameter('safety_distance', 0.6).value
        self.cone_radius = self.declare_parameter('cone_radius', 0.2).value

        # Speed Profile
        self.a_lat_max = self.declare_parameter('a_lat_max', 2.0).value
        self.v_max_straight = self.declare_parameter('v_max_straight', 7.0).value
        self.v_min = self.declare_parameter('v_min', 1.0).value

        self.path_processor = PathProcessor(PathProcessorConfig(
            a_lat_max=self.a_lat_max,
            v_max_straight=self.v_max_straight,
            v_min=self.v_min,
        ))

        # Noise Smoothing
        self.alpha = self.declare_parameter('alpha', 0.5).value

        self.get_logger().info(f"[MPPI] Using path topic: {self.path_topic}")

        self.sub_odom = self.create_subscription(
           Odometry, '/odom', self.odom_callback, 10)

        self.sub_path = self.create_subscription(
           Path, self.path_topic, self.path_callback, 10)

        self.sub_cones = self.create_subscription(
           PointCloud2, '/slam/cones', self.cone_callback, 10)

        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.pub_ackermann = self.create_publisher(
            AckermannDrive, '/ackermann_cmd', 10)

        self.pub_viz_path = self.create_publisher(
            Path, '/viz/mppi_path', 10)
        
        self.pub_params = self.create_publisher(
            String, '/mppi/parameters', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.vehicle_state = None
        self.path_arr = None
        self.path_heading = None
        self.path_v_ref = None
        self.obstacles = np.empty((0, 3))

        self.u0 = np.zeros((self.horizon, 2))
        self.u0[:, 0] = 1.0
        self.path_idx = 0

        self.get_logger().info("MPPI Controller Initialized")

    def odom_callback(self, msg):
        """ Update vehicle state from Odometry """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        
        # Convert Quaternion to Euler (Yaw)
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Get speed (assuming forward velocity is x-component of twist in base_link)
        v = msg.twist.twist.linear.x
        
        self.vehicle_state = np.array([p.x, p.y, yaw, v], dtype=float)

    def path_callback(self, msg):
        """ 
        Receive global path, calculate curvature and speed profile.
        Assumes path doesn't change drastically every frame.
        """
        n_points = len(msg.poses)
        if n_points < 2:
            return

        path_x = [p.pose.position.x for p in msg.poses]
        path_y = [p.pose.position.y for p in msg.poses]
        
        # Stack into (N, 2)
        new_path = np.column_stack((path_x, path_y))
        result = self.path_processor(new_path)

        # Update member variables
        self.path_arr = result.points
        self.path_v_ref = result.speed_ref
        self.path_heading = result.heading
        
        # Reset index if path changes significantly (optional logic)
        # For now, we rely on the search window in the control loop
        self.get_logger().info(f"Path received with {n_points} points.")

    def cone_callback(self, msg):
        """ Parse PointCloud2 cones into obstacle array """
        # Read points using sensor_msgs_py
        points_list = list(pc2.read_points(msg, field_names=("x", "y"),             skip_nans=True))
        if not points_list:
            return
            
        points_np = np.array(points_list) # (N, 2)
        
        # Add radius column
        r = self.cone_radius * np.ones((points_np.shape[0], 1))
        self.obstacles = np.hstack([points_np, r])

    def control_loop(self):
        """ Main MPPI Iteration (Called at 20Hz) """
        
        # 1. Check if ready
        if self.vehicle_state is None:
            self.get_logger().info("Waiting for vehicle state")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3))
            return
        if self.path_arr is None:
            self.get_logger().info("Waiting for path")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3))
            return
        
        x = self.vehicle_state.copy()
        N_path = self.path_arr.shape[0]

        # ------------------------
        # 2. Find closest path point (Search Window)
        # ------------------------
        # Search in a window around the previous index to be efficient
        search_window = self.search_window
        i0 = self.path_idx
        i1 = min(self.path_idx + search_window, N_path)
        
        # If we reached the end, wrap search or stop? 
        # Assuming open track for now, search end of array
        if i0 >= N_path - 1:
            i0 = 0
            i1 = search_window
            
        dists = np.linalg.norm(self.path_arr[i0:i1] - x[:2], axis=1)
        if len(dists) > 0:
            idx_local = np.argmin(dists)
            self.path_idx = i0 + idx_local
        else:
            self.path_idx = 0 # Fallback

        curr_idx = self.path_idx

        # ------------------------
        # 3. Get Reference Horizon
        # ------------------------
        # Extract slices for Path, Heading, Speed
        # Pad if horizon goes beyond path length
        indices = np.arange(curr_idx, curr_idx + self.horizon + 1)
        # Handle end of path (clamp to last point)
        indices_clamped = np.clip(indices, 0, N_path - 1)
        
        ref_pts = self.path_arr[indices_clamped]          # (self.horizon+1, 2)
        heading_ref = self.path_heading[indices_clamped[:-1]] # (self.horizon,) - heading is for steps 1..self.horizon
        v_ref = self.path_v_ref[indices_clamped]          # (self.horizon+1,)

        # ------------------------
        # 4. Sample Noise
        # ------------------------
        noise = np.random.randn(self.num_rollouts, self.horizon, 2) * self.sigma_u_base # Simplified noise for ROS speed
        
        # Smoothing (Time correlation)
        alpha = 0.5
        for h in range(1, self.horizon):
            noise[:, h, :] = self.alpha * noise[:, h-1, :] + (1 - self.alpha) * noise[:, h, :]
            
        U_rollouts = self.u0[None, :, :] + noise

        # ------------------------
        # 5. Simulate Dynamics (Rollouts)
        # ------------------------
        X_sim = np.zeros((self.num_rollouts, self.horizon+1, 4))
        X_sim[:, 0, :] = x  # Initialize all rollouts at current state
        X_sim = dynamics_vec(X_sim, U_rollouts, self.dt, self.max_accel, self.max_steer, self.min_vel, self.max_vel)

        # ------------------------
        # 6. Cost Calculation
        # ------------------------
        cost = np.zeros(self.num_rollouts)

        # A. Path Deviation
        d_path = np.linalg.norm(X_sim[:, :, :2] - ref_pts[None, :, :], axis=2)
        cost += self.w_path * np.sum(d_path[:, 1:], axis=1)

        # B. Heading Error
        sim_heading = X_sim[:, 1:, 2]
        # Normalize error
        h_err = sim_heading - heading_ref[None, :]
        h_err = np.arctan2(np.sin(h_err), np.cos(h_err))
        cost += self.w_heading * np.sum(np.abs(h_err), axis=1)

        # C. Control Effort
        cost += self.w_control * np.sum(U_rollouts**2, axis=(1, 2))

        # D. Terminal Cost
        cost += self.w_terminal * np.linalg.norm(X_sim[:, -1, :2] - ref_pts[-1], axis=1)

        # E. Speed Cost (Overspeed penalty)
        v_traj = X_sim[:, :, 3]
        speed_err = v_traj - v_ref[None, :]
        over_speed = np.clip(speed_err, 0.0, None)
        cost += self.w_speed * np.sum(over_speed[:, 1:]**2, axis=1)

        # F. Obstacle Cost
        if self.obstacles.shape[0] > 0:
            # Broadcast dimensions: (self.num_rollouts, self.horizon+1, 1, 2) - (1, 1, N_obs, 2)
            X_pos = X_sim[:, :, :2][:, :, None, :]
            obs_pos = self.obstacles[:, :2][None, None, :, :]
            obs_r = self.obstacles[:, 2][None, None, :]

            # Distance to all obstacles
            # Note: If N_obs is huge, this might be slow in Python.
            # Consider filtering obstacles by distance to car first.
            d_obs = np.linalg.norm(X_pos - obs_pos, axis=-1) - obs_r
            
            safety_dist = 0.6
            d_pen = self.safety_distance - d_obs
            d_pen = np.clip(d_pen, 0.0, None)
            
            obs_cost = self.w_obstacle * np.sum(d_pen**2, axis=(1, 2))
            cost += np.clip(obs_cost, 0.0, 1e4)

        # ------------------------
        # 7. Update Weights (MPPI)
        # ------------------------
        min_cost = cost.min()
        weights = np.exp(-(cost - min_cost) / self.lambda_)
        weights /= (weights.sum() + 1e-12)

        du = np.sum(weights[:, None, None] * noise, axis=0)
        self.u0 += du
        
        # Clamp controls
        self.u0[:, 0] = clamp(self.u0[:, 0], -self.max_accel, self.max_accel)
        self.u0[:, 1] = clamp(self.u0[:, 1], -self.max_steer, self.max_steer)

        # ------------------------
        # 8. Publish Control
        # ------------------------
        # Current optimal control
        accel_cmd = float(self.u0[0, 0])
        steer_cmd = float(self.u0[0, 1])
        
        # Speed command logic: 
        # Ackermann message usually takes speed, not acceleration.
        # We integrate accel for a target speed, or just send current v + dt*a
        target_speed = x[3] + self.dt * accel_cmd
        target_speed = clamp(target_speed, self.min_vel, self.max_vel)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steer_cmd
        drive_msg.drive.speed = target_speed
        drive_msg.drive.acceleration = accel_cmd
        
        self.pub_drive.publish(drive_msg)
        
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = steer_cmd
        ack_msg.speed = target_speed
        ack_msg.acceleration = accel_cmd
        self.pub_ackermann.publish(ack_msg)

        # ------------------------
        # 9. Shift & Debug
        # ------------------------
        # Visualize the best rollout (optional)
        # weighted_path = np.sum(weights[:, None, None] * X_sim[:, :, :2], axis=0)
        # self.publish_viz_path(weighted_path)

        # Shift controls for receding horizon
        self.u0 = np.vstack([self.u0[1:], np.zeros((1, 2))])

    def publish_viz_path(self, path_pts):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for pt in path_pts:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            msg.poses.append(pose)
        self.pub_viz_path.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = MPPIController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()