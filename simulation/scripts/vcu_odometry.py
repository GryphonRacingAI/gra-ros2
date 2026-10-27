#!/usr/bin/env python3
"""Odometry the way the real car can do it: wheel speeds (/vcu2ai) + IMU yaw rate (/imu/data).

Publishes /odom (odom -> base_link) and the matching TF. Nothing here is sim-only, so it can run
on the car too. In sim, /ground_truth/odom (Gazebo's exact pose) is used only to start at the
spawn pose, so the odom frame lines up with the world at t=0; after that the estimate drifts like
a real one (wheel slip, IMU noise). The gyro bias is estimated while the car stands still.

source:=ground_truth republishes /ground_truth/odom instead (perfect odometry, for debugging).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from fsai_api.msg import VCU2AI


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class VcuOdometry(Node):
    def __init__(self):
        super().__init__('vcu_odometry')
        self.source = self.declare_parameter('source', 'wheel').value
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.2575).value)
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        rate = float(self.declare_parameter('rate', 100.0).value)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/ground_truth/odom', self.ground_truth_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.gyro_bias, self.bias_sum, self.bias_n = 0.0, 0.0, 0
        self.v = self.w = 0.0
        self.started = False
        self.last_imu = None
        self.stamp = None
        if self.source == 'wheel':
            # Deep queues: with the 5-message sensor-data default, a 20 ms stall of this node at the
            # IMU's 300 Hz dropped samples, and a 0.5 s gap mid-corner lost 27 deg of heading.
            buffered = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(VCU2AI, '/vcu2ai', self.vcu_cb, 100)
            self.create_subscription(Imu, '/imu/data', self.imu_cb, buffered)
            self.create_timer(1.0 / rate, self.publish)
            # no ground truth (real car): start at the origin after 2 s
            self.create_timer(2.0, self.start_without_ground_truth)
        self.get_logger().info(f'odometry source: {self.source}')

    def start_without_ground_truth(self):
        if not self.started:
            self.started = True
            self.get_logger().info('no /ground_truth/odom: odometry starts at the origin')

    def ground_truth_cb(self, msg):
        if self.source == 'ground_truth':
            self.odom_pub.publish(msg)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.send_tf(msg.header.stamp, p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        elif not self.started:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.x, self.y = p.x, p.y
            self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.started = True
            self.get_logger().info(f'odometry starts at the spawn pose ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})')

    def vcu_cb(self, msg):
        rpm = (msg.fl_wheel_speed_rpm + msg.fr_wheel_speed_rpm + msg.rl_wheel_speed_rpm + msg.rr_wheel_speed_rpm) / 4.0
        self.v = rpm * 2.0 * math.pi / 60.0 * self.wheel_radius

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = msg.angular_velocity.z
        # Gyro bias: learnt while the car stands still (wheels at 0), as cars do before the start.
        # Uncorrected, a MEMS bias of ~0.01 deg/s turned into several degrees of heading per run.
        if abs(self.v) < 1e-3:
            self.bias_sum += raw
            self.bias_n += 1
            if self.bias_n >= 50:
                self.gyro_bias = self.bias_sum / self.bias_n
        w_prev, self.w = self.w, raw - self.gyro_bias
        if self.started and self.last_imu is not None:
            dt = t - self.last_imu
            if 0.0 < dt < 1.0:
                # trapezoid: also bridges the odd missing samples without losing the rotation
                dyaw = 0.5 * (w_prev + self.w) * dt
                mid = self.yaw + dyaw / 2.0
                self.x += self.v * math.cos(mid) * dt
                self.y += self.v * math.sin(mid) * dt
                self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
            elif dt >= 1.0:
                self.get_logger().warn(f'no IMU data for {dt:.1f} s: heading not integrated over the gap',
                                       throttle_duration_sec=5.0)
        self.last_imu = t
        self.stamp = msg.header.stamp

    def publish(self):
        if not self.started or self.stamp is None:
            return
        m = Odometry()
        m.header.stamp = self.stamp
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self.x, self.y
        q = yaw_quat(self.yaw)
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = q
        m.twist.twist.linear.x = self.v
        m.twist.twist.angular.z = self.w
        self.odom_pub.publish(m)
        self.send_tf(self.stamp, self.x, self.y, self.yaw)

    def send_tf(self, stamp, x, y, yaw):
        if not self.publish_tf:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        q = yaw_quat(yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VcuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
