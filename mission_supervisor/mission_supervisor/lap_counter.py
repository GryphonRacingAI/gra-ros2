#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from common_msgs.msg import ConeArray
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
import math
import copy

# --- Time Import for TF Fix ---
from builtin_interfaces.msg import Time

# --- TF2 Imports ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
import tf2_geometry_msgs  

class LapCounter(Node):
    def __init__(self):
        super().__init__('lap_counter')

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Explicitly define our frames here for clean architecture
        self.target_frame = 'map'       # The global frame we want to calculate laps in
        self.cone_frame = 'velodyne'    # The frame the cones are published in
        self.odom_frame = 'odom'        # The frame the car's odometry is published in

        # Subscribers
        self.cone_sub = self.create_subscription(
            ConeArray,
            '/cone_array',
            self.cone_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  
            self.odom_callback,
            10
        )

        # Publisher
        self.lap_counter_pub = self.create_publisher(UInt16, '/laps', 10)

        # State
        self.car_position = Point()
        self.cone_positions = []
        self.last_known_cones = None
        self.previous_position = None
        self.lap_count = 0 
        self.in_cooldown = False

        # Timer for periodic publishing
        self.publish_timer = self.create_timer(1.0, self.publish_lap_count)

        self.get_logger().info(f"Lap Counter node initialised. Tracking laps in [{self.target_frame}] frame.")

    def cone_callback(self, msg):

        # Fallback to msg.header.frame_id if it exists, otherwise force 'velodyne'
        source_frame = msg.header.frame_id if hasattr(msg, 'header') and msg.header.frame_id else self.cone_frame

        transformed_cones = []
        for cone in msg.large_orange_cones[:2]:
            try:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = source_frame
                
                # FIX: Use Time() to ask for the most recent available transform
                point_stamped.header.stamp = Time() 
                point_stamped.point = cone.position

                # TRANSFORM: velodyne -> map
                transformed = self.tf_buffer.transform(
                    point_stamped,
                    self.target_frame,
                    timeout=Duration(seconds=0.1) 
                )

                transformed_cone = copy.deepcopy(cone)
                transformed_cone.position = transformed.point
                transformed_cones.append(transformed_cone)

            except TransformException as ex:
                # We downgrade this to a debug print so it doesn't spam if a single frame drops
                self.get_logger().debug(f"Could not transform cone from {source_frame} to {self.target_frame}: {ex}")
                return 
        
        if len(transformed_cones) == 2:
            self.last_known_cones = transformed_cones

    def odom_callback(self, msg):
        # We must also transform the car's position to 'map' so it shares a coordinate 
        # system with our transformed cones.
        try:
            car_pt = PointStamped()
            car_pt.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.odom_frame
            
            # FIX: Use Time() to ask for the most recent available transform
            car_pt.header.stamp = Time()
            car_pt.point = msg.pose.pose.position
            
            # TRANSFORM: odom -> map
            transformed_car = self.tf_buffer.transform(
                car_pt,
                self.target_frame,
                timeout=Duration(seconds=0.1)
            )
            self.car_position = transformed_car.point
            
        except TransformException as ex:
            self.get_logger().debug(f"Could not transform car pose from {car_pt.header.frame_id} to {self.target_frame}: {ex}")
            return

        if self.previous_position is not None and self.last_known_cones is not None and not self.in_cooldown:
            first_cone = self.last_known_cones[0]
            second_cone = self.last_known_cones[1]

            finish_line_vector = (second_cone.position.x - first_cone.position.x, second_cone.position.y - first_cone.position.y)
            prev_car_vector = (self.previous_position.x - first_cone.position.x, self.previous_position.y - first_cone.position.y)
            curr_car_vector = (self.car_position.x - first_cone.position.x, self.car_position.y - first_cone.position.y)
            
            prev_cross = finish_line_vector[0] * prev_car_vector[1] - finish_line_vector[1] * prev_car_vector[0]
            curr_cross = finish_line_vector[0] * curr_car_vector[1] - finish_line_vector[1] * curr_car_vector[0]
            
            if prev_cross * curr_cross < 0:  
                self.get_logger().info("Crossing detected! Checking proximity to finish line...")
                if self.is_near_finish_line(first_cone, second_cone, self.car_position):
                    self.lap_count += 1
                    self.get_logger().info(f"Lap completed! Total laps: {self.lap_count}")
                    self.initiate_cooldown()
                else:
                    self.get_logger().info("Crossing detected but car is too far from finish line, ignoring.")
       
        self.previous_position = self.car_position

    def is_near_finish_line(self, first_cone, second_cone, car_position, threshold=4.0):
        def point_line_distance(px, py, x1, y1, x2, y2):
            line_mag = math.dist((x1, y1), (x2, y2))
            if line_mag < 1e-6:
                return float('inf')  
            u1 = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_mag**2)
            u = max(min(u1, 1), 0)
            ix = x1 + u * (x2 - x1)
            iy = y1 + u * (y2 - y1)
            return math.dist((px, py), (ix, iy))

        distance = point_line_distance(
            car_position.x, car_position.y, 
            first_cone.position.x, first_cone.position.y, 
            second_cone.position.x, second_cone.position.y
        )
        return distance <= threshold

    def initiate_cooldown(self):
        self.in_cooldown = True
        self.get_logger().info("Cooldown initiated.")
        self.cooldown_timer = self.create_timer(10.0, self.end_cooldown) 

    def end_cooldown(self):
        self.in_cooldown = False
        self.get_logger().info("Cooldown ended. Ready to detect laps again.")
        self.cooldown_timer.cancel() 

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