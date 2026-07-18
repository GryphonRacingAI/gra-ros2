#!/usr/bin/env python3
"""
Static Inspection A Mission Node – EUFS ros_can version (2026)
- Uses eufs_msgs + standard ros_can topics (observed from /ros_can/*)
- AckermannDriveStamped on /cmd_vel_out (header stamped – required by ros_can bridge)
- Mission flag → /ros_can/mission_completed
- Brake kept (common in EUFS setups); wheel speeds & states split for clarity
"""

"""
eufs_msgs/msg/CanState
# State of the Autonomous System
uint16 as_state
uint16 AS_OFF=0
uint16 AS_READY=1
uint16 AS_DRIVING=2
uint16 AS_EMERGENCY_BRAKE=3
uint16 AS_FINISHED=4
# Mission indicator
uint16 ami_state
uint16 AMI_NOT_SELECTED=10
uint16 AMI_ACCELERATION=11
uint16 AMI_SKIDPAD=12
uint16 AMI_AUTOCROSS=13
uint16 AMI_TRACK_DRIVE=14
uint16 AMI_AUTONOMOUS_DEMO=15
uint16 AMI_ADS_INSPECTION=16
uint16 AMI_ADS_EBS=17
uint16 AMI_DDT_INSPECTION_A=18
uint16 AMI_DDT_INSPECTION_B=19
uint16 AMI_JOYSTICK=20
uint16 AMI_MANUAL=21
"""

'''
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
eufs_msgs/WheelSpeeds speeds
	float32 steering
	float32 lf_speed
	float32 rf_speed
	float32 lb_speed
	float32 rb_speed

'''

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Header
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState, WheelSpeedsStamped   # ← change here if your msg is different

class StaticInspectionA(Node):
    AS_DRIVING = 2
    AMI_STATIC_INSPECTION_A = 18
    WHEEL_RADIUS = 0.2575
    AXLE_SPEED_RPM = 200

    def __init__(self):
        super().__init__('static_inspection_A')

        # Publishers – ros_can style
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/cmd', 10)                     # control
        self.mission_flag_pub = self.create_publisher(
            Bool, '/ros_can/mission_flag', 1)                      # mission complete
        self.driving_flag_pub = self.create_publisher(
            Bool, '/state_machine/driving_flag', 1)                # enable cmd

        # Subscribers – ros_can topics
        self.state_sub = self.create_subscription(
            CanState, '/ros_can/state', self.state_callback, 10)
        self.wheel_sub = self.create_subscription(
            WheelSpeedsStamped, '/ros_can/wheel_speeds', self.wheel_callback, 10)

        # State storage
        self.as_state = None
        self.ami_state = None
        self.lb_speed = 0.0
        self.rb_speed = 0.0
        self._last_as = None
        self._last_ami = None

        self.mission_started = False
        self.mission_complete = False
        self.conditions_met = False

        self.target_speed_mps = (
            self.AXLE_SPEED_RPM * (2 * math.pi) / 60.0 * self.WHEEL_RADIUS
        )

        self.get_logger().info("StaticInspectionA (ros_can edition) initialized")
        self.get_logger().info(f"Target = {self.target_speed_mps:.3f} m/s ({self.AXLE_SPEED_RPM} rpm)")
        self.get_logger().info("Waiting for AS_DRIVING + AMI_STATIC_INSPECTION_A on /ros_can/state ...")

    def state_callback(self, msg: CanState):
        self.as_state = msg.as_state          
        self.ami_state = msg.ami_state

        # Exact same logging style as your original autonomous_demo.py
        if self._last_as is not None and self._last_as != self.as_state:
            self.get_logger().info(f"AS State changed to: {self.as_state}")
        if self._last_ami is not None and self._last_ami != self.ami_state:
            self.get_logger().info(f"AMI State changed to: {self.ami_state}")

        self._last_as = self.as_state
        self._last_ami = self.ami_state

        # Mission gate (exactly as before)
        if (self.as_state == self.AS_DRIVING and
            self.ami_state == self.AMI_STATIC_INSPECTION_A and
            not self.mission_started):
            if not self.conditions_met:
                self.conditions_met = True
                self.get_logger().info("Gate open → Starting Static Inspection A")
                self.start_mission()

        # on as_ready 
        if (self.as_state == self.AS)

    def wheel_callback(self, msg: WheelSpeedsStamped):
        # Adjust field names to what you see with ros2 topic echo
        self.rl_wheel_speed_rpm = msg.rear_left   
        self.rr_wheel_speed_rpm = msg.rear_right

    def publish_drive(self, speed: float = 0.0, steering: float = 0.0):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.cmd_publisher.publish(msg)

    def start_mission(self):
        if self.mission_started: return
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
                self.publish_drive(steering=target)
                time.sleep(dt)

    def ramp_up_drivetrain(self):
        self.get_logger().info(f"Ramping to {self.target_speed_mps:.3f} m/s over 10 s")
        ramp_duration = 10.0
        hold_duration = 5.0
        dt = 0.1
        t0 = time.time()
        while rclpy.ok() and not self.mission_complete:
            elapsed = time.time() - t0
            if elapsed > ramp_duration: break
            speed = self.linear_interpolate(elapsed, 0.0, ramp_duration, 0.0, self.target_speed_mps)
            self.publish_drive(speed=speed)
            time.sleep(dt)

        self.get_logger().info(f"Holding target for {hold_duration} s")
        t_hold = time.time()
        while rclpy.ok() and not self.mission_complete and (time.time() - t_hold) < hold_duration:
            self.publish_drive(speed=self.target_speed_mps)
            time.sleep(dt)

        self.stop_drivetrain()
        self.signal_completion()

    def stop_drivetrain(self):
        self.publish_drive(speed=0.0)
        self.get_logger().info("Drivetrain stopped")

    def signal_completion(self):
        self.get_logger().info("Applying brake + waiting for wheel speeds < 5 rpm...")
        brake_msg = Bool(data=True)
        self.brake_publisher.publish(brake_msg)
        self.get_logger().info("Brake applied (True)")

        while rclpy.ok() and not self.mission_complete:
            if self.rl_wheel_speed_rpm < 5 and self.rr_wheel_speed_rpm < 5:
                self.get_logger().info(f"Wheels stopped ({self.rl_wheel_speed_rpm:.1f}, {self.rr_wheel_speed_rpm:.1f}) → Mission complete")
                break
            self.get_logger().info(f"Waiting... RL={self.rl_wheel_speed_rpm:.1f} RR={self.rr_wheel_speed_rpm:.1f}")
            time.sleep(0.1)

        self.brake_publisher.publish(Bool(data=False))
        self.get_logger().info("Brake released")

        # EUFS standard flag
        flag = Bool(data=True)
        self.mission_completed_publisher.publish(flag)
        self.get_logger().info("Mission completed flag raised – done")

    def emergency_stop(self):
        self.get_logger().warn("Emergency stop")
        self.emergency_brake_publisher.publish(Bool(data=True))
        self.publish_drive(speed=0.0)

    @staticmethod
    def linear_interpolate(value, a1, a2, b1, b2):
        if value <= a1: return b1
        if value >= a2: return b2
        return b1 + (b2 - b1) * (value - a1) / (a2 - a1)

def main(args=None):
    rclpy.init(args=args)
    node = StaticInspectionA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(speed=0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()