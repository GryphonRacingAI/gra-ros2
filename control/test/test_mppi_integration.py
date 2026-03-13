#!/usr/bin/env python3
import os
import sys
import time
import math
import unittest

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
import rclpy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Quaternion

INNER_CONES_CSV = os.path.join(
    os.path.dirname(__file__), '..', '..', 'simulation', 'tracks',
    'mppi_track', 'inner_cones.csv'
)
OUTER_CONES_CSV = os.path.join(
    os.path.dirname(__file__), '..', '..', 'simulation', 'tracks',
    'mppi_track', 'outer_cones.csv'
)


def generate_test_description():
    mppi_node = launch_ros.actions.Node(
        package='control',
        executable='mppi_ros_modified.py',
        name='mppi_controller',
        parameters=[{
            'test_mode': 'static_test',
            'inner_cones_csv': os.path.realpath(INNER_CONES_CSV),
            'outer_cones_csv': os.path.realpath(OUTER_CONES_CSV),
            'path_topic': '/path',
        }],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            mppi_node,
            launch.actions.TimerAction(
                period=2.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {'mppi_controller': mppi_node},
    )


class TestMPPIController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mppi')

    def tearDown(self):
        self.node.destroy_node()

    def _publish_odom(self, x=7.5, y=44.0, yaw=0.0, v=1.0):
        pub = self.node.create_publisher(Odometry, '/odom', 10)
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        half_yaw = yaw / 2.0
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw)
        )
        msg.twist.twist.linear.x = v

        rclpy.spin_once(self.node, timeout_sec=0.5)
        pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.5)
        return pub

    def test_publishes_drive(self, proc_output):
        msgs_rx = []
        sub = self.node.create_subscription(
            AckermannDriveStamped, '/drive',
            lambda msg: msgs_rx.append(msg), 100,
        )
        try:
            odom_pub = self._publish_odom()
            end_time = time.time() + 15
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if msgs_rx:
                    break
            assert len(msgs_rx) > 0, "No AckermannDriveStamped messages received on /drive"
        finally:
            self.node.destroy_subscription(sub)

    def test_drive_values_reasonable(self, proc_output):
        msgs_rx = []
        sub = self.node.create_subscription(
            AckermannDriveStamped, '/drive',
            lambda msg: msgs_rx.append(msg), 100,
        )
        try:
            odom_pub = self._publish_odom()
            end_time = time.time() + 15
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if len(msgs_rx) >= 3:
                    break

            assert len(msgs_rx) >= 1, "No drive messages received"

            max_steer = math.pi / 6.0
            for msg in msgs_rx:
                steer = msg.drive.steering_angle
                speed = msg.drive.speed
                assert -max_steer - 1e-6 <= steer <= max_steer + 1e-6, \
                    f"Steering {steer} out of bounds [-{max_steer}, {max_steer}]"
                assert -1.0 - 1e-6 <= speed <= 8.0 + 1e-6, \
                    f"Speed {speed} out of bounds [-1.0, 8.0]"
        finally:
            self.node.destroy_subscription(sub)


@launch_testing.post_shutdown_test()
class TestMPPIControllerShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
