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

class TrackPathfinder(Node):
    def __init__(self):
        super().__init__('track_pathfinder')

        # get event type
        self.declare_parameter('event', 'trackdrive')
        event = self.get_parameter('event').get_parameter_value().string_value
        if event.lower() == 'acceleration':
            mission_type = MissionTypes.acceleration
        elif event.lower() == 'skidpad':
            mission_type = MissionTypes.skidpad
        elif event.lower() == 'autocross':
            mission_type = MissionTypes.autocross
        elif event.lower() == 'trackdrive':
            mission_type = MissionTypes.trackdrive
        else:
            self.get_logger().error(f"Invalid mission_type: '{event}'. Defaulting to 'acceleration'.")
            mission_type = MissionTypes.acceleration
        
        self.path_planner = PathPlanner(mission_type) # acceleration, skidpad, autocross, trackdrive
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.create_subscription(ConeArray, '/cone_array', self.cone_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.final_lap = False
        self.final_path_published = False
        self.latest_odom = None
        self.get_logger().info("TrackPathfinder node initialized.")

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