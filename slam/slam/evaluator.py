#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from common_msgs.msg import ConeArray
import math
import numpy as np



def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class EvaluatorNode(Node):
    def __init__(self):
        super().__init__("slam_evaluator")
        # Pose Subscribers 
        self.slam_pose_sub = Subscriber(self,PoseStamped, "/slam/pose")   
        self.perfect_odom_sub = Subscriber(self,Odometry, "/odom")
        # Pose topics Synchronizer
        self.pose_sync = ApproximateTimeSynchronizer([self.slam_pose_sub, self.perfect_odom_sub], queue_size=20, slop=0.05) 
        self.pose_sync.registerCallback(self.pose_callback)
        self.get_logger().info("Pose Subscribers synchronized. Waiting for data...")
        # Map Subscribers 
        self.slam_map_sub = Subscriber(self,ConeArray, "/slam/cone_map")   
        self.perfect_cone_map_sub = Subscriber(self,ConeArray, "/perfect_cone_map")
        # Map topics Synchronizer
        self.map_sync = ApproximateTimeSynchronizer([self.slam_map_sub, self.perfect_cone_map_sub], queue_size=20, slop=0.05) 
        self.map_sync.registerCallback(self.map_callback)
        self.get_logger().info("Cone Map Subscribers synchronized. Waiting for data...")

        # Storage Arrays
        self.position_errors = []
        self.yaw_errors = []
        self.landmark_errors = []
        self.total_landmarks = []
        self.matched_landmarks = []
        self.initial_slam_pos = None
        self.initial_perfect_pos = None
        self.get_logger().info("SLAM Evaluator Started")
        
    # POSE EVALUATION
    def pose_callback(self, slam_pose, perfect_odom):
        # Extract raw positions from topics
        xs_raw = slam_pose.pose.position.x
        ys_raw = slam_pose.pose.position.y
        xp_raw = perfect_odom.pose.pose.position.x
        yp_raw = perfect_odom.pose.pose.position.y
        # Save the very first pose as the "zero" reference
        if self.initial_slam_pos is None:
            self.initial_slam_pos = (xs_raw, ys_raw)
            self.initial_perfect_pos = (xp_raw, yp_raw)
        # 3. Shift both trajectories to start exactly at (0,0)
        xs = xs_raw - self.initial_slam_pos[0]
        ys = ys_raw - self.initial_slam_pos[1]
        xp = xp_raw - self.initial_perfect_pos[0]
        yp = yp_raw - self.initial_perfect_pos[1]
    
        pose_error = math.sqrt((xs - xp)**2 + (ys - yp)**2)
        self.position_errors.append(pose_error)

        yaw_s = quaternion_to_yaw(slam_pose.pose.orientation)
        yaw_p = quaternion_to_yaw(perfect_odom.pose.pose.orientation)
        angle = yaw_s - yaw_p
        yaw_error = (angle + math.pi) % (2 * math.pi) - math.pi
        self.yaw_errors.append(abs(yaw_error))

    # MAP EVALUATION
    def map_callback(self, slam_map, perfect_map):
        # Extracting slam cone points
        slam_cone_points = list()
        for cone_group in [slam_map.yellow_cones, slam_map.blue_cones, slam_map.orange_cones, slam_map.large_orange_cones]:
            for cone in cone_group:
                slam_cone_points.append([cone.position.x, cone.position.y])
        # Extracting perfect_cone points
        perfect_cone_points = list()
        for cone_group in [perfect_map.yellow_cones, perfect_map.blue_cones, perfect_map.orange_cones, perfect_map.large_orange_cones]:
            for cone in cone_group:
                perfect_cone_points.append([cone.position.x, cone.position.y])
        if len(perfect_cone_points)==0:
            self.get_logger().info("Perfect Cone Array is empty.")
            return
        
        total_error = 0
        matches = 0
        slam_cone_points = np.array(slam_cone_points)   #convering to numpy for using array operations
        for p_point in perfect_cone_points:
            if len(slam_cone_points) == 0:
                continue
            distances = np.linalg.norm(slam_cone_points - p_point, axis=1)
            min_dist = np.min(distances)
            if min_dist<0.5: #(0.5m threshold)
                matches += 1
                total_error += min_dist
        if matches > 0:
            mean_error = total_error/matches
            self.landmark_errors.append(mean_error)
            self.total_landmarks.append(len(perfect_cone_points))
            self.matched_landmarks.append(matches)
            if len(self.total_landmarks)%20 == 0:
                self.print_map_metrics()
    def print_map_metrics(self):
        if len(self.landmark_errors) == 0:
            return
        mean_landmark_error = np.mean(self.landmark_errors)
        detection_rate = np.sum(self.matched_landmarks) / np.sum(self.total_landmarks) * 100.0
        self.get_logger().info(
            f"[MAP] Mean Landmark Error: {mean_landmark_error:.3f} m | "
            f"Detection Rate: {detection_rate:.1f}%"
        )
    
    def destroy_node(self):
        if len(self.position_errors) > 0:
            rmse_pos = np.sqrt(np.mean(np.square(self.position_errors)))
            rmse_yaw = np.sqrt(np.mean(np.square(self.yaw_errors)))
            self.get_logger().info("========== FINAL RESULTS ==========")
            self.get_logger().info(f"Final Position RMSE: {rmse_pos:.3f} m")
            self.get_logger().info(f"Final Yaw RMSE: {math.degrees(rmse_yaw):.2f} deg")

        if len(self.landmark_errors) > 0:
            mean_landmark_error = np.mean(self.landmark_errors)
            detection_rate = (np.sum(self.matched_landmarks) / np.sum(self.total_landmarks)) * 100.0
            self.get_logger().info(f"Final Landmark Error: {mean_landmark_error:.3f} m")
            self.get_logger().info(f"Final Detection Rate: {detection_rate:.1f}%")
        return super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAM Evaluation node shut down by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()