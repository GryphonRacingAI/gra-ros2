#!/usr/bin/env python3
"""
Static Inspection B Mission Node (ROS 2)

- Ramp drivetrain to 50 rpm (open-loop)
- Trigger EBS → VCU should enter AS_EMERGENCY_BRAKE
"""

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from fsai_api.msg import VCU2AI


class StaticInspectionB(Node):
    # AS / AMI constants (kept for clarity)
    AS_DRIVING = 3
    AMI_STATIC_INSPECTION_B = 6

    WHEEL_RADIUS = 0.2575
    AXLE_SPEED_RPM = 50

    def __init__(self):
        super().__init__('static_inspection_B')

        self.ackermann_publisher = self.create_publisher(
            AckermannDrive, '/ackermann_cmd_controller', 1)
        self.emergency_brake_publisher = self.create_publisher(
            Bool, '/emergency_brake', 1)
        self.chequered_flag_publisher = self.create_publisher(
            Bool, '/chequered_flag', 1)

        self.vcu2ai_subscriber = self.create_subscription(
            VCU2AI, '/vcu2ai', self.vcu2ai_callback, 1)

        None
        self.ami_state = None
        self.mission_started = False
        self.mission_complete = False

        self.target_speed_mps = (
            self.AXLE_SPEED_RPM * (2 * math.pi) / 60 * self.WHEEL_RADIUS
        )

        self.get_logger().info("StaticInspectionB initialized")
        self.get_logger().info(f"Target: {self.target_speed_mps:.3f} m/s ({self.AXLE_SPEED_RPM} rpm)")
        self.get_logger().info("Waiting for mission gate...")

    def vcu2ai_callback(self, msg: VCU2AI):
        self.as_state = msg.as_state
        self.ami_state = msg.ami_state

        if (self.as_state == self.AS_DRIVING and
            self.ami_state == self.AMI_STATIC_INSPECTION_B and
            not self.mission_started):

            self.mission_started = True
            self.get_logger().info("Gate open → Starting Static Inspection B")
            self.start_mission()

    def start_mission(self):
        try:
            time.sleep(2.0)                    # settle
            self.ramp_up_drivetrain()
            time.sleep(0.5)
            self.trigger_emergency_brake()
            self.mission_complete = True
        except Exception as e:
            self.get_logger().error(f"Mission error: {e}")
            self.emergency_stop()

    def ramp_up_drivetrain(self):
        self.get_logger().info("Starting 50 rpm ramp")
        ramp_duration = 5.0
        dt = 0.1
        t0 = time.time()

        while rclpy.ok() and not self.mission_complete:
            elapsed = time.time() - t0
            if elapsed > ramp_duration + 1.0:
                break

            speed = self.linear_interpolate(elapsed, 0.0, ramp_duration, 0.0, self.target_speed_mps)

            msg = AckermannDrive()
            msg.speed = float(speed)
            msg.steering_angle = 0.0
            self.ackermann_publisher.publish(msg)

            if int(elapsed * 2) % 2 == 0:   # log every 0.5s
                self.get_logger().info(f"Speed: {speed:.3f} m/s")
            time.sleep(dt)

    def trigger_emergency_brake(self):
        self.get_logger().info("Triggering EBS")
        self.emergency_brake_publisher.publish(Bool(data=True))

        stop = AckermannDrive(speed=0.0, steering_angle=0.0)
        self.ackermann_publisher.publish(stop)

        self.chequered_flag_publisher.publish(Bool(data=True))

    def emergency_stop(self):
        self.get_logger().warn("Emergency stop")
        self.emergency_brake_publisher.publish(Bool(data=True))
        self.ackermann_publisher.publish(AckermannDrive(speed=0.0, steering_angle=0.0))

    @staticmethod
    def linear_interpolate(value, a1, a2, b1, b2):
        if value <= a1:
            return b1
        if value >= a2:
            return b2
        return b1 + (b2 - b1) * (value - a1) / (a2 - a1)


def main(args=None):
    rclpy.init(args=args)
    node = StaticInspectionB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
