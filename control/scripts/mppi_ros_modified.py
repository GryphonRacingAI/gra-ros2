#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

# ROS Messages
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Header

# TF transformations
from tf_transformations import euler_from_quaternion
import sensor_msgs_py.point_cloud2 as pc2

# ============================
# Parameters & Constants
# ============================
DT = 0.05
H = 16          # Horizon
K = 200         # Number of rollouts
LAMBDA = 2.0
SIGMA_U_BASE = np.array([0.6, 0.15])
SIGMA_U_MIN  = np.array([0.2, 0.05])

# Weights
W_PATH = 20.0
W_HEADING = 3.0
W_SPEED = 10.0
W_CONTROL = 1.5
W_TERMINAL = 5.0
W_OBSTACLE = 15.0

# Limits
MAX_A = 3.0
MIN_V = -1.0
MAX_V = 8.0
MAX_STEER = np.pi/6  # ~0.52 rad

# Vehicle specific
L_BASE = 1.6 # Wheelbase (Approximate, used for better kinematic model if needed)

# Helper Functions
def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def normalize_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

def dynamics_vec(X, U):
    """
    Vectorized dynamics:
    X: (K, H+1, 4)  [x, y, theta, v]
    U: (K, H, 2)    [accel, steer_angle]
    """
    K_curr, H_curr, _ = U.shape
    
    # Note: X is mutable and updated in place for next steps
    for h in range(H_curr):
        a = clamp(U[:, h, 0], -MAX_A, MAX_A)
        steer = clamp(U[:, h, 1], -MAX_STEER, MAX_STEER)
        
        theta = X[:, h, 2]
        v = X[:, h, 3]

        # Standard Kinematic Bicycle Model Update
        # dx = v * cos(theta) * dt
        # dy = v * sin(theta) * dt
        # dtheta = v * tan(steer) / L * dt  (Standard) 
        # OR dtheta = steer * dt (Simplified model from original script)
        
        # Using the Simplified Model from original script to preserve behavior:
        X[:, h+1, 0] = X[:, h, 0] + DT * v * np.cos(theta)
        X[:, h+1, 1] = X[:, h, 1] + DT * v * np.sin(theta)
        X[:, h+1, 2] = theta + DT * steer # Note: This treats steer as yaw rate
        X[:, h+1, 3] = clamp(v + DT * a, MIN_V, MAX_V)

    return X

class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_controller')
        
        self.path_topic = self.declare_parameter('path_topic', '/perfect_path').value
        self.ask_path_topic = self.declare_parameter('ask_path_topic', False).value

        if self.ask_path_topic and sys.stdin.isatty():
            ans = input("Choose path topic: [1] /path  [2] /perfect_path (default 1): ").strip()
            if ans == "2":
                self.path_topic = "/perfect_path"
            else:
                self.path_topic = "/path"

        self.get_logger().info(f"[MPPI] Using path topic: {self.path_topic}")

        # -- Subscribers --
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.sub_path = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10)
            
        self.sub_cones = self.create_subscription(
            PointCloud2, '/slam/cones', self.cone_callback, 10)

        # -- Publishers --
        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
            
        self.pub_ackermann = self.create_publisher(
            AckermannDrive, '/ackermann_cmd', 10)
            
        # Optional: Publish the predicted trajectories for visualization
        self.pub_viz_path = self.create_publisher(
            Path, '/viz/mppi_path', 10)

        # -- Timer --
        self.timer = self.create_timer(DT, self.control_loop)

        # -- State Variables --
        self.vehicle_state = None  # [x, y, theta, v]
        self.path_arr = None       # np.array [[x,y], ...]
        self.path_heading = None
        self.path_v_ref = None
        self.obstacles = np.empty((0, 3)) # [x, y, radius]

        # MPPI Internal State
        self.u0 = np.zeros((H, 2))
        self.u0[:, 0] = 1.0  # bias forward
        self.path_idx = 0    # Closest index on path
        self.initialized = False

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
        
        # Compute Gradients for Reference Speed (Same as script)
        dx_p = np.gradient(path_x)
        dy_p = np.gradient(path_y)
        ddx_p = np.gradient(dx_p)
        ddy_p = np.gradient(dy_p)

        # Curvature kappa
        kappa = (dx_p * ddy_p - dy_p * ddx_p) / (dx_p**2 + dy_p**2 + 1e-6)**1.5
        kappa_abs = np.abs(kappa)

        # Speed Profile Generation
        a_lat_max = 2.0
        v_max_straight = 7.0
        v_min = 1.0
        
        v_ref = np.sqrt(a_lat_max / (kappa_abs + 1e-3))
        v_ref = np.clip(v_ref, v_min, v_max_straight)
        
        # Heading
        heading_ref = np.arctan2(dy_p, dx_p)

        # Update member variables
        self.path_arr = new_path
        self.path_v_ref = v_ref
        self.path_heading = heading_ref
        
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
        
        # Add radius column (0.2m)
        r = 0.2 * np.ones((points_np.shape[0], 1))
        self.obstacles = np.hstack([points_np, r])

    def control_loop(self):
        """ Main MPPI Iteration (Called at 20Hz) """
        
        # 1. Check if ready
        if self.vehicle_state is None or self.path_arr is None:
            if self.vehicle_state is None:
                self.get_logger().info("Waiting for vehicle state")
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
        search_window = 50
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
        indices = np.arange(curr_idx, curr_idx + H + 1)
        # Handle end of path (clamp to last point)
        indices_clamped = np.clip(indices, 0, N_path - 1)
        
        ref_pts = self.path_arr[indices_clamped]          # (H+1, 2)
        heading_ref = self.path_heading[indices_clamped[:-1]] # (H,) - heading is for steps 1..H
        v_ref = self.path_v_ref[indices_clamped]          # (H+1,)

        # ------------------------
        # 4. Sample Noise
        # ------------------------
        noise = np.random.randn(K, H, 2) * SIGMA_U_BASE # Simplified noise for ROS speed
        
        # Smoothing (Time correlation)
        alpha = 0.5
        for h in range(1, H):
            noise[:, h, :] = alpha * noise[:, h-1, :] + (1 - alpha) * noise[:, h, :]
            
        U_rollouts = self.u0[None, :, :] + noise

        # ------------------------
        # 5. Simulate Dynamics (Rollouts)
        # ------------------------
        X_sim = np.zeros((K, H+1, 4))
        X_sim[:, 0, :] = x  # Initialize all rollouts at current state
        X_sim = dynamics_vec(X_sim, U_rollouts)

        # ------------------------
        # 6. Cost Calculation
        # ------------------------
        cost = np.zeros(K)

        # A. Path Deviation
        d_path = np.linalg.norm(X_sim[:, :, :2] - ref_pts[None, :, :], axis=2)
        cost += W_PATH * np.sum(d_path[:, 1:], axis=1)

        # B. Heading Error
        sim_heading = X_sim[:, 1:, 2]
        # Normalize error
        h_err = sim_heading - heading_ref[None, :]
        h_err = np.arctan2(np.sin(h_err), np.cos(h_err))
        cost += W_HEADING * np.sum(np.abs(h_err), axis=1)

        # C. Control Effort
        cost += W_CONTROL * np.sum(U_rollouts**2, axis=(1, 2))

        # D. Terminal Cost
        cost += W_TERMINAL * np.linalg.norm(X_sim[:, -1, :2] - ref_pts[-1], axis=1)

        # E. Speed Cost (Overspeed penalty)
        v_traj = X_sim[:, :, 3]
        speed_err = v_traj - v_ref[None, :]
        over_speed = np.clip(speed_err, 0.0, None)
        cost += W_SPEED * np.sum(over_speed[:, 1:]**2, axis=1)

        # F. Obstacle Cost
        if self.obstacles.shape[0] > 0:
            # Broadcast dimensions: (K, H+1, 1, 2) - (1, 1, N_obs, 2)
            X_pos = X_sim[:, :, :2][:, :, None, :]
            obs_pos = self.obstacles[:, :2][None, None, :, :]
            obs_r = self.obstacles[:, 2][None, None, :]

            # Distance to all obstacles
            # Note: If N_obs is huge, this might be slow in Python.
            # Consider filtering obstacles by distance to car first.
            d_obs = np.linalg.norm(X_pos - obs_pos, axis=-1) - obs_r
            
            safety_dist = 0.6
            d_pen = safety_dist - d_obs
            d_pen = np.clip(d_pen, 0.0, None)
            
            obs_cost = W_OBSTACLE * np.sum(d_pen**2, axis=(1, 2))
            cost += np.clip(obs_cost, 0.0, 1e4)

        # ------------------------
        # 7. Update Weights (MPPI)
        # ------------------------
        min_cost = cost.min()
        weights = np.exp(-(cost - min_cost) / LAMBDA)
        weights /= (weights.sum() + 1e-12)

        du = np.sum(weights[:, None, None] * noise, axis=0)
        self.u0 += du
        
        # Clamp controls
        self.u0[:, 0] = clamp(self.u0[:, 0], -MAX_A, MAX_A)
        self.u0[:, 1] = clamp(self.u0[:, 1], -MAX_STEER, MAX_STEER)

        # ------------------------
        # 8. Publish Control
        # ------------------------
        # Current optimal control
        accel_cmd = float(self.u0[0, 0])
        steer_cmd = float(self.u0[0, 1])
        
        # Speed command logic: 
        # Ackermann message usually takes speed, not acceleration.
        # We integrate accel for a target speed, or just send current v + dt*a
        target_speed = x[3] + DT * accel_cmd
        target_speed = clamp(target_speed, MIN_V, MAX_V)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
