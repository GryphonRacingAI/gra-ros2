#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from common_msgs.msg import ConeArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from fsd_path_planning import PathPlanner, MissionTypes
from fsd_path_planning.sorting_cones.core_cone_sorting import ConeSorting
from fsd_path_planning.cone_matching.core_cone_matching import ConeMatching
from fsd_path_planning.calculate_path.core_calculate_path import CalculatePath

class TrackPathfinder(Node):
    def __init__(self):
        super().__init__('track_pathfinder')

        self.declare_parameter('experimental_performance_improvements', False)
        # Cone Sorting
        self.declare_parameter('max_n_neighbors', 5)
        self.declare_parameter('max_dist', 6.5)
        self.declare_parameter('max_dist_to_first', 6.0)
        self.declare_parameter('max_length', 12)
        self.declare_parameter('threshold_directional_angle_deg', 40.0)
        self.declare_parameter('threshold_absolute_angle_deg', 65.0)
        self.declare_parameter('use_unknown_cones', True)
        # Cone Matching
        self.declare_parameter('min_track_width', 3.0)
        self.declare_parameter('max_search_range', 5.0)
        self.declare_parameter('max_search_angle_deg', 50.0)
        self.declare_parameter('matches_should_be_monotonic', False)
        # Pathing
        self.declare_parameter('maximal_distance_for_valid_path', 5.0)
        self.declare_parameter('mpc_path_length', 20.0)
        self.declare_parameter('mpc_prediction_horizon', 40)
        self.declare_parameter('smoothing', 0.2)
        self.declare_parameter('predict_every', 0.1)
        self.declare_parameter('max_deg', 3)

        # Always use trackdrive: live cone sort/match/centreline, no mission
        # relocalizer (acceleration/skidpad skip sorting and freeze a map path).
        experimental = self.get_parameter('experimental_performance_improvements').value
        self.path_planner = PathPlanner(MissionTypes.trackdrive, experimental)
        self._configure_path_planner(experimental)

        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.create_subscription(ConeArray, '/cone_array', self.cone_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.final_lap = False
        self.final_path_published = False
        self.latest_odom = None
        self.get_logger().info(
            "TrackPathfinder initialized (mission=trackdrive, body frame: car at origin, +x forward)."
        )

    def _configure_path_planner(self, experimental):
        self.path_planner.cone_sorting = ConeSorting(
            max_n_neighbors=self.get_parameter('max_n_neighbors').value,
            max_dist=self.get_parameter('max_dist').value,
            max_dist_to_first=self.get_parameter('max_dist_to_first').value,
            max_length=self.get_parameter('max_length').value,
            threshold_directional_angle=np.deg2rad(
                self.get_parameter('threshold_directional_angle_deg').value
            ),
            threshold_absolute_angle=np.deg2rad(
                self.get_parameter('threshold_absolute_angle_deg').value
            ),
            use_unknown_cones=self.get_parameter('use_unknown_cones').value,
            experimental_performance_improvements=experimental,
        )

        self.path_planner.cone_matching = ConeMatching(
            min_track_width=self.get_parameter('min_track_width').value,
            max_search_range=self.get_parameter('max_search_range').value,
            max_search_angle=np.deg2rad(
                self.get_parameter('max_search_angle_deg').value
            ),
            matches_should_be_monotonic=self.get_parameter(
                'matches_should_be_monotonic'
            ).value,
        )

        self.path_planner.pathing = CalculatePath(
            smoothing=self.get_parameter('smoothing').value,
            predict_every=self.get_parameter('predict_every').value,
            maximal_distance_for_valid_path=self.get_parameter(
                'maximal_distance_for_valid_path'
            ).value,
            max_deg=self.get_parameter('max_deg').value,
            mpc_path_length=self.get_parameter('mpc_path_length').value,
            mpc_prediction_horizon=self.get_parameter('mpc_prediction_horizon').value,
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def cone_callback(self, msg):
        """
        1. Organize cones by type (order: unknown, yellow, blue, orange, large_orange) 
        2. Input this into fsd_path_planning.PathPlanner
        """        

        # velodyne -> odom
        # 

        global_cones_temp = [
            np.array([[cone.position.x, cone.position.y] for cone in msg.unknown_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.yellow_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.blue_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.orange_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.large_orange_cones], dtype=np.float32).reshape(-1, 2),
        ]
        global_cones = global_cones_temp  # Preserve fixed order: [unknown, yellow, blue, orange, large_orange]

        if all(c.shape[0] == 0 for c in global_cones):
            self.get_logger().warn("All cone arrays are empty, skipping path planning.")
            return

        car_position = np.array([0.0, 0.0])
        car_direction = np.array([1.0, 0.0])

        # Calculate the path
        try:
            path = self.path_planner.calculate_path_in_global_frame(global_cones, car_position, car_direction)
            self.publish_path(path)
        except Exception as e:
            self.get_logger().warn(f"Path planning failed: {e}")

    def publish_path(self, path):
        if path is None or len(path) < 5:
            self.get_logger().warn("Path is empty or too short, not publishing.")
            return

        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = 'velodyne'

        # Start from the 5th waypoint, skipping the first four
        for point in path[4:]:
            # Defensive: check point shape and type
            if len(point) < 3 or not all(isinstance(x, (float, np.floating, int)) for x in point[1:3]):
                self.get_logger().warn(f"Skipping invalid path point: {point}")
                continue
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(point[1])
            pose.pose.position.y = float(point[2])
            pose.pose.position.z = 0.0
            ros_path.poses.append(pose)

        if not ros_path.poses:
            self.get_logger().warn("No valid poses in path, not publishing.")
            return

        self.path_pub.publish(ros_path)

def main(args=None):
    rclpy.init(args=args)
    node = TrackPathfinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()