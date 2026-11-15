#!/usr/bin/env python3
"""Record a simulator run for score_run.py.

usage: ros2 run simulation gt_record.py out.csv        (stop with Ctrl-C when the run is over)

Writes out.csv with one row per /ground_truth/odom message (Gazebo's exact pose of the car):
    t, x, y, v, laps, state, z, roll, pitch, odom_v, yaw
(laps from /laps, state from /mission_state, odom_v from /odom) and out_slam.csv with every /slam/pose:
    t, x, y, yaw
"""
import math
import sys

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt16

from fsai_common.geometry import yaw_from_quaternion


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, path):
        super().__init__('gt_record', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt = open(path, 'w')
        self.gt.write('t,x,y,v,laps,state,z,roll,pitch,odom_v,yaw\n')
        self.slam = open(path[:-4] + '_slam.csv' if path.endswith('.csv') else path + '_slam.csv', 'w')
        self.slam.write('t,x,y,yaw\n')
        self.laps, self.state, self.odom_v, self.rows = 0, '', 0.0, 0
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_ground_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(UInt16, '/laps', self.on_laps, 10)
        self.create_subscription(String, '/mission_state', self.on_state, 10)
        self.create_subscription(PoseStamped, '/slam/pose', self.on_slam, 10)

    def on_odom(self, msg):
        self.odom_v = msg.twist.twist.linear.x

    def on_laps(self, msg):
        self.laps = msg.data

    def on_state(self, msg):
        self.state = msg.data

    def on_slam(self, msg):
        p = msg.pose.position
        self.slam.write(f'{stamp_s(msg.header):.3f},{p.x:.3f},{p.y:.3f},{yaw_from_quaternion(msg.pose.orientation):.4f}\n')
        self.slam.flush()

    def on_ground_truth(self, msg):
        p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.gt.write(f'{stamp_s(msg.header):.3f},{p.x:.3f},{p.y:.3f},{math.hypot(v.x, v.y):.3f},{self.laps},{self.state},'
                      f'{p.z:.3f},{roll:.3f},{pitch:.3f},{self.odom_v:.2f},{yaw_from_quaternion(q):.3f}\n')
        self.rows += 1
        if self.rows % 20 == 0:
            self.gt.flush()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rclpy.init()
    node = Recorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.gt.close()
    node.slam.close()


if __name__ == '__main__':
    main()
