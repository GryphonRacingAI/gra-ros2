#!/usr/bin/env python3

# This node counts the laps by drawing a line connecting large orange cones and detecting cross product sign change
# Updated to use cone_array in odom frame for consistent coordinate system

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from common_msgs.msg import ConeArray
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
import math

class LapCounter(Node):
    def __init__(self):
        super().__init__('lap_counter')

        # Subscribers
        self.cone_sub = self.create_subscription(
            ConeArray,
            '/cone_array',  # cone_array is in odom frame
            self.cone_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # odometry in odom frame
            self.odom_callback,
            10
        )

        # Publisher
        self.lap_counter_pub = self.create_publisher(UInt16, '/laps', 10)

        # State
        self.car_position = Point()
        self.cone_positions = []
        self.previous_position = None
        self.lap_count = 0  # Lap count
        self.in_cooldown = False

        # Timer for periodic publishing
        self.publish_timer = self.create_timer(1.0, self.publish_lap_count)

        self.get_logger().info("Lap Counter node initialised (using odom frame coordinates)")

    def cone_callback(self, msg):
        # Extract large orange cones from the ConeArray message (odom frame)
        self.cone_positions = msg.large_orange_cones

    def odom_callback(self, msg):
        # Odometry is in odom frame, same as cones - perfect coordinate consistency
        self.car_position = msg.pose.pose.position

        if self.previous_position is not None and len(self.cone_positions) >= 2 and not self.in_cooldown:
            first_cone = self.cone_positions[0]
            second_cone = self.cone_positions[1]

            # Calculate vectors - using .position since cones have position field
            finish_line_vector = (second_cone.position.x - first_cone.position.x, second_cone.position.y - first_cone.position.y)
            prev_car_vector = (self.previous_position.x - first_cone.position.x, self.previous_position.y - first_cone.position.y)
            curr_car_vector = (self.car_position.x - first_cone.position.x, self.car_position.y - first_cone.position.y)

            # Calculate cross products to determine if the car has crossed the finish line
            prev_cross = finish_line_vector[0] * prev_car_vector[1] - finish_line_vector[1] * prev_car_vector[0]
            curr_cross = finish_line_vector[0] * curr_car_vector[1] - finish_line_vector[1] * curr_car_vector[0]

            if prev_cross * curr_cross < 0:  # Signs are different, indicating a crossing
                if self.is_near_finish_line(first_cone, second_cone, self.car_position):
                    self.lap_count += 1
                    self.get_logger().info(f"Lap completed! Total laps: {self.lap_count}")
                    self.initiate_cooldown()

        self.previous_position = self.car_position

    def is_near_finish_line(self, first_cone, second_cone, car_position, threshold=4.0):    # 4.0 meter away from the line segment will be ignored
        """Check if the car is near the line segment formed by the first and second cone."""
        def point_line_distance(px, py, x1, y1, x2, y2):
            # Calculate the distance from point (px, py) to the line segment (x1, y1) - (x2, y2)
            line_mag = math.dist((x2, x1), (y2, y1))
            if line_mag < 1e-6:
                return float('inf')  # Line segment too small, effectively a point
            u1 = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_mag**2)
            u = max(min(u1, 1), 0)
            ix = x1 + u * (x2 - x1)
            iy = y1 + u * (y2 - y1)
            return math.dist((px, py), (ix, iy))

        # Using .position field of cones (odom frame coordinates)
        distance = point_line_distance(car_position.x, car_position.y, 
                                     first_cone.position.x, first_cone.position.y, 
                                     second_cone.position.x, second_cone.position.y)
        self.get_logger().info(f"Distance from car to finish line: {distance}")
        return distance <= threshold

    def initiate_cooldown(self):
        self.in_cooldown = True
        self.get_logger().info("Cooldown initiated.")
        self.cooldown_timer = self.create_timer(10.0, self.end_cooldown)  # 10 second cooldown period

    def end_cooldown(self):
        self.in_cooldown = False
        self.get_logger().info("Cooldown ended. Ready to detect laps again.")
        self.cooldown_timer.cancel()  # Cancel the timer after cooldown ends

    def publish_lap_count(self):
        msg = UInt16()
        msg.data = self.lap_count
        self.lap_counter_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lap_counter = LapCounter()
    
    try:
        rclpy.spin(lap_counter)
    except KeyboardInterrupt:
        pass
    finally:
        lap_counter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
