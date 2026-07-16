#!/usr/bin/env python3
"""
Static Inspection A Mission Node (ROS 2 / rclpy port) – Updated

- Wheel-speed check now uses < 5 rpm (per your latest request)
- Before raising chequered flag:
  1. Publish brake = True
  2. Wait until BOTH wheels < 5 rpm
- State-change logging kept (as in autonomous_demo.py)
"""

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from fsai_api.msg import VCU2AI


class StaticInspectionA(Node):
    # AS / AMI constants
    AS_DRIVING = 3
    AMI_STATIC_INSPECTION_A = 5

    WHEEL_RADIUS = 0.2575
    AXLE_SPEED_RPM = 200

    def __init__(self):
        super().__init__('static_inspection_A')

        # Publishers
        self.ackermann_publisher = self.create_publisher(
            AckermannDrive, '/ackermann_cmd_controller', 1)
        self.chequered_flag_publisher = self.create_publisher(
            Bool, '/chequered_flag', 1)
        self.brake_publisher = self.create_publisher(Bool, '/brake', 1)
        self.emergency_brake_publisher = self.create_publisher(
            Bool, '/emergency_brake', 1)

        # Subscriber + wheel speed storage
        self.vcu2ai_subscriber = self.create_subscription(
            VCU2AI, '/vcu2ai', self.vcu2ai_callback, 1)

        self.as_state = None
        self.ami_state = None
        self.rl_wheel_speed_rpm = None
        self.rr_wheel_speed_rpm = None
        self._last_as_state = None
        self._last_ami_state = None
    
        self.brake_press_f_pct = None 
        self.brake_press_r_pct = None

        self.mission_started = False
        self.mission_complete = False
        self.conditions_met = False

        self.target_speed_mps = (
            self.AXLE_SPEED_RPM * (2.0 * math.pi) / 60.0 * self.WHEEL_RADIUS
        )

        self.get_logger().info("StaticInspectionA initialized")
        self.get_logger().info(
            f"Target ramp = {self.target_speed_mps:.3f} m/s "
            f"({self.AXLE_SPEED_RPM} rpm)"
        )
        self.get_logger().info("Waiting for AS_DRIVING + AMI_STATIC_INSPECTION_A ...")

    def vcu2ai_callback(self, msg: VCU2AI):
        self.as_state = msg.as_state
        self.ami_state = msg.ami_state
        self.rl_wheel_speed_rpm = msg.rl_wheel_speed_rpm
        self.rr_wheel_speed_rpm = msg.rr_wheel_speed_rpm

        # === AS/AMI state change logging (exactly as in autonomous_demo.py) ===
        if (hasattr(self, '_last_as_state') and
            self._last_as_state is not None and
            self._last_as_state != self.as_state):
            self.get_logger().info(f"AS State changed to: {self.as_state}")

        if (hasattr(self, '_last_ami_state') and
            self._last_ami_state is not None and
            self._last_ami_state != self.ami_state):
            self.get_logger().info(f"AMI State changed to: {self.ami_state}")

        self._last_as_state = self.as_state
        self._last_ami_state = self.ami_state
        # =====================================================================

        # Mission gate
        if (self.as_state == self.AS_DRIVING and
            self.ami_state == self.AMI_STATIC_INSPECTION_A and
            not self.mission_started):

            if not self.conditions_met:
                self.conditions_met = True
                self.get_logger().info("Gate open → Starting Static Inspection A")
                self.start_mission()

    def start_mission(self):
        if self.mission_started:
            return
        self.mission_started = True

        try:
            time.sleep(3.0)
            self.sweep_steering()
            time.sleep(1.0)
            self.ramp_up_drivetrain()
            self.mission_complete = True
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.emergency_stop()

    def sweep_steering(self):
        self.get_logger().info("Starting steering sweep")
        angles = [-0.7, 0.7, 0.0]
        hold_time = 3.0
        dt = 0.1
        for target in angles:
            self.get_logger().info(f"Steering → {target}")
            t0 = time.time()
            while rclpy.ok() and (time.time() - t0) < hold_time:
                msg = AckermannDrive(steering_angle=float(target))
                self.ackermann_publisher.publish(msg)
                time.sleep(dt)

    def ramp_up_drivetrain(self):
        self.get_logger().info(f"Ramping to {self.target_speed_mps:.3f} m/s over 10 s")
        ramp_duration = 10.0
        hold_duration = 5.0
        dt = 0.1
        t0 = time.time()

        # Ramp
        while rclpy.ok() and not self.mission_complete:
            elapsed = time.time() - t0
            if elapsed > ramp_duration:
                break
            speed = self.linear_interpolate(elapsed, 0.0, ramp_duration, 0.0, self.target_speed_mps)
            msg = AckermannDrive(speed=float(speed), steering_angle=0.0)
            self.ackermann_publisher.publish(msg)
            time.sleep(dt)

        # Hold
        self.get_logger().info(f"Holding target speed for {hold_duration} s")
        t_hold = time.time()
        while rclpy.ok() and not self.mission_complete and (time.time() - t_hold) < hold_duration:
            msg = AckermannDrive(speed=float(self.target_speed_mps), steering_angle=0.0)
            self.ackermann_publisher.publish(msg)
            time.sleep(dt)

        self.stop_drivetrain()
        self.signal_completion()

    def stop_drivetrain(self):
        msg = AckermannDrive(speed=0.0, steering_angle=0.0)
        self.ackermann_publisher.publish(msg)
        self.get_logger().info("Drivetrain stopped")

    def signal_completion(self):
        """Before raising flag:
           1. Apply brake (publish brake=True)
           2. Wait until BOTH wheels < 5 rpm
           3. Stop applying brake
           4. 
        """
        self.get_logger().info("Applying brake + waiting for wheel speeds < 5 rpm...")

        # Step 1: Apply brake
        brake_msg = Bool(data=True)
        self.brake_publisher.publish(brake_msg)
        self.get_logger().info("Brake applied (True)")

        # Step 2: Wait for wheels to stop
        while rclpy.ok() and not self.mission_complete:
            rl = self.rl_wheel_speed_rpm
            rr = self.rr_wheel_speed_rpm

            if rl is not None and rr is not None and rl < 5 and rr < 5:
                self.get_logger().info(f"Wheel speeds OK ({rl:.1f}, {rr:.1f} rpm) → Raising flag")
                break

            self.get_logger().info(f"Waiting... RL={rl} rpm, RR={rr} rpm")
            time.sleep(0.1)

        # Step 1: Stop brake apply
        brake_msg = Bool(data=False)
        self.brake_publisher.publish(brake_msg)
        self.get_logger().info("Brake applied (False)")

        time.sleep(0.5)

        # Step 3: Raise chequered flag
        flag = Bool(data=True)
        self.chequered_flag_publisher.publish(flag)
        self.get_logger().info("Chequered flag raised – mission complete")

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
    node = StaticInspectionA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_drivetrain()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()