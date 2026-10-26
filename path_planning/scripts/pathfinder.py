#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from common_msgs.msg import ConeArray


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def map_to_base_link(points, rx, ry, yaw):
    """Transform (N, 2) points from map frame to base_link (car-local).

    Car pose (rx, ry, yaw) must be in map. +x is forward, +y is left.
    Uses the same rotation as fastslam_node._process_cones.
    """
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    dx = pts[:, 0] - rx
    dy = pts[:, 1] - ry
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


def base_link_to_odom(points, ox, oy, yaw):
    """Transform (N, 2) points from base_link to odom frame.

    Car pose (ox, oy, yaw) is from /odom. Output matches MPPI vehicle_state frame.
    """
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    out_x = ox + pts[:, 0] * c - pts[:, 1] * s
    out_y = oy + pts[:, 0] * s + pts[:, 1] * c
    return np.column_stack((out_x, out_y))


def base_link_to_map(points, rx, ry, yaw):
    """Transform (N, 2) points from base_link to map frame using /slam/pose."""
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    out_x = rx + pts[:, 0] * c - pts[:, 1] * s
    out_y = ry + pts[:, 0] * s + pts[:, 1] * c
    return np.column_stack((out_x, out_y))


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
        
        # Last 2 Previously published /path values
        self.path_buffer : np.ndarray

    def calculate_path(self, yellow_cones, blue_cones):
        """
        Calculate centreline using geometric midpoint + smoothing
        Input: yellow_cones, blue_cones as [[x, y], ...] in base_link (car-local)
        Output: smoothed waypoints [[x, y], ...] in base_link or None
        
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

        order = np.argsort(filtered[:, 0])
        return filtered[order]

    def _predict_next_midpoint(self) -> np.ndarray:
        """
        Takes last 2 midpoints gives next midpoint, assuming no directional change
        """
        if len(self.path_buffer) < 2:
            return errno.EINVAL
        p0 = self.path_buffer[0]
        p1 = self.path_buffer[1]
        return 2*p0 - p1
            

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
    def __init__(self):
        super().__init__('centreline_track_pathfinder')

        self.output_frame = self.declare_parameter('output_frame', 'odom').value
        self.cone_map_topic = self.declare_parameter('cone_map_topic', '/slam/cone_map').value
        lookahead = self.declare_parameter('lookahead_distance', 20.0).value
        self.test_mode = bool(self.declare_parameter('test_mode', True).value)
        self.viz_frame = self.declare_parameter('viz_frame', 'map').value
        self.marker_scale = float(self.declare_parameter('marker_scale', 0.25).value)
        self.line_width = float(self.declare_parameter('line_width', 0.15).value)

        self.slam_pose = None
        self.odom_pose = None
        self.last_odom_stamp = None

        self.create_subscription(ConeArray, self.cone_map_topic, self.cone_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/slam/pose', self.slam_pose_callback, 10)

        self.get_logger().info(
            f"PATHFINDER: map cones -> base_link planning -> {self.output_frame} path"
        )
        if self.test_mode:
            self.get_logger().info(
                f"PATHFINDER: test_mode enabled, publishing /viz/path and "
                f"/viz/path_markers in {self.viz_frame}"
            )

        self.centreline_planner = CentrelineAlgorithm(self.get_logger(), lookahead_distance=lookahead)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.viz_path_pub = self.create_publisher(Path, '/viz/path', 10) if self.test_mode else None
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/path_markers', 1) if self.test_mode else None

    def slam_pose_callback(self, msg):
        q = msg.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.slam_pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self.last_odom_stamp = msg.header.stamp

    def cone_callback(self, msg):
        if self.slam_pose is None or self.odom_pose is None:
            self.get_logger().warn("[PATHFINDER] Waiting for /slam/pose and /odom")
            return

        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f"[PATHFINDER] Expected cone_map in map frame, got '{msg.header.frame_id}'"
            )

        yellow_map = [[c.position.x, c.position.y] for c in msg.yellow_cones]
        blue_map = [[c.position.x, c.position.y] for c in msg.blue_cones]

        rx, ry, r_yaw = self.slam_pose
        yellow_local = map_to_base_link(yellow_map, rx, ry, r_yaw)
        blue_local = map_to_base_link(blue_map, rx, ry, r_yaw)

        self.get_logger().info(
            f"[PATHFINDER INPUT] Yellow: {len(yellow_local)}, Blue: {len(blue_local)}, "
            f"cone_map frame: {msg.header.frame_id}"
        )
        if len(yellow_local) > 0:
            self.get_logger().info(
                f"  Yellow local - X: [{yellow_local[:, 0].min():.2f}, {yellow_local[:, 0].max():.2f}], "
                f"Y: [{yellow_local[:, 1].min():.2f}, {yellow_local[:, 1].max():.2f}]"
            )
        if len(blue_local) > 0:
            self.get_logger().info(
                f"  Blue local - X: [{blue_local[:, 0].min():.2f}, {blue_local[:, 0].max():.2f}], "
                f"Y: [{blue_local[:, 1].min():.2f}, {blue_local[:, 1].max():.2f}]"
            )

        try:
            path_local = self.centreline_planner.calculate_path(
                yellow_local.tolist(), blue_local.tolist()
            )

            if path_local is not None:
                ox, oy, o_yaw = self.odom_pose
                path_out = base_link_to_odom(path_local, ox, oy, o_yaw)
                self.get_logger().info(
                    f"[PATHFINDER OUTPUT] {len(path_out)} waypoints in {self.output_frame}"
                )
                self.get_logger().info(
                    f"  Range - X: [{path_out[:, 0].min():.2f}, {path_out[:, 0].max():.2f}], "
                    f"Y: [{path_out[:, 1].min():.2f}, {path_out[:, 1].max():.2f}]"
                )
                self.get_logger().info(
                    f"  First 5: {[f'({p[0]:.2f}, {p[1]:.2f})' for p in path_out[:5]]}"
                )
                self.publish_path(path_out, self.output_frame, msg.header.stamp)
                if self.test_mode:
                    path_viz = base_link_to_map(path_local, rx, ry, r_yaw)
                    self.publish_viz(path_viz, self.viz_frame, msg.header.stamp)
            else:
                self.get_logger().warn(
                    "[PATHFINDER OUTPUT] No valid path generated (insufficient cones or waypoints)"
                )
        except Exception as e:
            self.get_logger().error(f"Planning Error: {e}")
    
    def _path_stamp(self, preferred_stamp):
        if preferred_stamp.sec != 0 or preferred_stamp.nanosec != 0:
            return preferred_stamp
        if self.last_odom_stamp is not None:
            return self.last_odom_stamp
        return self.get_clock().now().to_msg()

    def _make_path_msg(self, waypoints, frame_id, stamp):
        ros_path = Path()
        ros_path.header.stamp = self._path_stamp(stamp)
        ros_path.header.frame_id = frame_id
        for point in waypoints:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            ros_path.poses.append(pose)
        return ros_path

    def publish_path(self, waypoints, frame_id, stamp):
        self.path_pub.publish(self._make_path_msg(waypoints, frame_id, stamp))

    def publish_viz(self, waypoints, frame_id, stamp):
        ros_path = self._make_path_msg(waypoints, frame_id, stamp)
        self.viz_path_pub.publish(ros_path)
        self.publish_path_markers(ros_path)

    def publish_path_markers(self, ros_path: Path):
        stamp = ros_path.header.stamp
        frame_id = ros_path.header.frame_id
        arr = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = stamp
        delete_all.ns = 'pathfinder'
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        points = Marker()
        points.header.frame_id = frame_id
        points.header.stamp = stamp
        points.ns = 'pathfinder'
        points.id = 1
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.scale.x = self.marker_scale
        points.scale.y = self.marker_scale
        points.scale.z = self.marker_scale
        points.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
        for pose in ros_path.poses:
            pt = Point()
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y
            pt.z = 0.1
            points.points.append(pt)
        arr.markers.append(points)

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = 'pathfinder'
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.line_width
        line.color = ColorRGBA(r=0.9, g=0.2, b=0.9, a=0.9)
        for pose in ros_path.poses:
            pt = Point()
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y
            pt.z = 0.1
            line.points.append(pt)
        arr.markers.append(line)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CentrelineTrackPathfinder()
    if not node.get_parameter('use_sim_time').value:
        node.get_logger().warn(
            "use_sim_time is false; in Gazebo set --ros-args -p use_sim_time:=true "
            "and launch RViz the same way to avoid TF timestamp errors"
        )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()