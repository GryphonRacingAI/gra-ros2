#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from common_msgs.msg import ConeArray

class CentrelineAlgorithm:
    """
    Centreline Pathfinder.
    geometric midpoint with path smoothing.
    """
    def __init__(self, logger, lookahead_distance: float = 20.0):
        self.logger = logger
        self.lookahead_distance = lookahead_distance
        
        self.min_x_lookahead = 0.5
        self.max_gate_distance = 8.0
        self.smoothing_window = 3
        self.cone_vertical_tolerance = 1.5 

    def calculate_path(self, yellow_cones, blue_cones):
        """
        Calculate centreline using geometric midpoint + smoothing
        Input: yellow_cones, blue_cones as [[x, y], [x, y], ...]
        Output: smoothed waypoints [[x, y], [x, y], ...] or None
        
        Algorithm:
        1. Filter cones ahead of car
        2. Sort by forward distance
        3. Greedily pair nearest Yellow-Blue cones
        4. Compute midpoints as centreline candidates
        5. Smooth waypoints *need to check if needed*
        """
        
        if len(yellow_cones) == 0 and len(blue_cones) == 0:
            return None
        
        y_np = np.array(yellow_cones) if len(yellow_cones) > 0 else np.empty((0, 2))
        b_np = np.array(blue_cones) if len(blue_cones) > 0 else np.empty((0, 2))
        
        # Filter & sort cones ahead of car
        y_filtered = self._filter_and_sort_cones(y_np)
        b_filtered = self._filter_and_sort_cones(b_np)

        if len(y_filtered) == 0 or len(b_filtered) == 0:
            return None

        # greedy cone sorting - for each sorted yellow, find closest blue
        midpoints = self._pair_and_midpoint(y_filtered, b_filtered)
        
        if len(midpoints) < 2:
            return None

        # Build waypoint list starting from car
        waypoints = [np.array([0.0, 0.0])]
        waypoints.extend(midpoints)

        waypoints = self._smooth_path(waypoints)
        waypoints = [wp for wp in waypoints if wp[0] >= 0.0] # ahead of the car
        
        if len(waypoints) < 2:
            return None

        return np.array(waypoints)

    def _filter_and_sort_cones(self, cones):
        """Filter cones ahead of car and sort by forward distance"""
        if len(cones) == 0:
            return np.empty((0, 2))
        
        # x > min_lookahead and within lookahead distance
        mask = (cones[:, 0] > self.min_x_lookahead) & \
               (np.linalg.norm(cones, axis=1) < self.lookahead_distance)
        filtered = cones[mask]
        
        if len(filtered) == 0:
            return np.empty((0, 2))
        
        # sort by forward distance based on x coord
        sorted_cones = filtered[np.argsort(filtered[:, 0])]
        return sorted_cones

    def _pair_and_midpoint(self, yellow_cones, blue_cones):
        """
        Pair yellow and blue cones to form gates.
        Strategy: For each yellow cone (in forward order), find closest blue cone.
        Returns list of midpoint waypoints.
        """
        midpoints = []
        blue_used = set()
        
        for y in yellow_cones:
            min_dist = float('inf')
            closest_b_idx = -1
            
            # Find closest unused blue cone
            for b_idx, b in enumerate(blue_cones):
                if b_idx in blue_used:
                    continue
                
                lateral_dist = abs(b[1] - y[1])
                
                # Check vertical tolerance
                vertical_diff = abs(b[0] - y[0])
                if vertical_diff > self.cone_vertical_tolerance:
                    continue
                
                if lateral_dist < min_dist:
                    min_dist = lateral_dist
                    closest_b_idx = b_idx
            
            # Accept pairing if within track width
            if closest_b_idx >= 0 and min_dist < self.max_gate_distance:
                b = blue_cones[closest_b_idx]
                midpoint = (y + b) / 2.0
                midpoints.append(midpoint)
                blue_used.add(closest_b_idx)
        
        return midpoints

    def _smooth_path(self, waypoints, window_size=None):
        """
        Smooth waypoints using moving average filter
        """
        if window_size is None:
            window_size = self.smoothing_window
        
        if len(waypoints) <= window_size:
            return waypoints
        
        waypoints = np.array(waypoints)
        smoothed = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            start = max(0, i - window_size // 2)
            end = min(len(waypoints), i + window_size // 2 + 1)
            
            window = waypoints[start:end]
            smoothed_point = np.mean(window, axis=0)
            smoothed.append(smoothed_point)
        
        smoothed.append(waypoints[-1])
        return smoothed

class CentrelineTrackPathfinder(Node):
    """ 
    Pathfinder
    """
    
    def __init__(self):
        super().__init__('centreline_track_pathfinder')

        self.create_subscription(ConeArray, '/slam/cone_map', self.cone_callback, 10)
        
        self.get_logger().info("PATHFINDER: Centreline Algorithm (Car-Local Coordinates)")
        
        self.centreline_planner = CentrelineAlgorithm(self.get_logger(), lookahead_distance=20.0)
        self.path_pub = self.create_publisher(Path, '/path', 10)
    
    def cone_callback(self, msg):

        #Separate Cones by Color (car-local coordinates)
        yellow_cones = [[c.position.x, c.position.y] for c in msg.yellow_cones]
        blue_cones = [[c.position.x, c.position.y] for c in msg.blue_cones]
        
        self.get_logger().info(f"[PATHFINDER INPUT] Yellow cones: {len(yellow_cones)}, Blue cones: {len(blue_cones)}, Frame: {msg.header.frame_id}")
        if len(yellow_cones) > 0:
            self.get_logger().info(f"  Yellow range - X: [{min(y[0] for y in yellow_cones):.2f}, {max(y[0] for y in yellow_cones):.2f}], Y: [{min(y[1] for y in yellow_cones):.2f}, {max(y[1] for y in yellow_cones):.2f}]")
        if len(blue_cones) > 0:
            self.get_logger().info(f"  Blue range - X: [{min(b[0] for b in blue_cones):.2f}, {max(b[0] for b in blue_cones):.2f}], Y: [{min(b[1] for b in blue_cones):.2f}, {max(b[1] for b in blue_cones):.2f}]")
        
        # Plan Path (in car coordinates)
        try:
            path_points = self.centreline_planner.calculate_path(yellow_cones, blue_cones)
            
            if path_points is not None:
                self.get_logger().info(f"[PATHFINDER OUTPUT] Generated {len(path_points)} waypoints, Frame: {msg.header.frame_id}")
                self.get_logger().info(f"  Waypoint range - X: [{min(p[0] for p in path_points):.2f}, {max(p[0] for p in path_points):.2f}], Y: [{min(p[1] for p in path_points):.2f}, {max(p[1] for p in path_points):.2f}]")
                self.get_logger().info(f"  First 5 waypoints: {[f'({p[0]:.2f}, {p[1]:.2f})' for p in path_points[:5]]}")
                self.publish_path(path_points, msg.header.frame_id)
            else:
                self.get_logger().warn("[PATHFINDER OUTPUT] No valid path generated (insufficient cones or waypoints)")
        except Exception as e:
            self.get_logger().error(f"Planning Error: {e}")
    
    def publish_path(self, waypoints, frame_id):
        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = frame_id

        for point in waypoints:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

def main(args=None):
    rclpy.init(args=args)
    node = CentrelineTrackPathfinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()