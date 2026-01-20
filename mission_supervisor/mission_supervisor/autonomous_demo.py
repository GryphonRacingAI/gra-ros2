#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from fsai_api.msg import VCU2AI

class AutonomousDemoStateWait(Node):
    # Constants from VCU2AI message
    AS_OFF = 1
    AS_READY = 2
    AS_DRIVING = 3
    AS_EMERGENCY_BRAKE = 4
    AS_FINISHED = 5
    
    AMI_NOT_SELECTED = 0
    AMI_ACCELERATION = 1
    AMI_SKIDPAD = 2
    AMI_AUTOCROSS = 3
    AMI_TRACK_DRIVE = 4
    AMI_STATIC_INSPECTION_A = 5
    AMI_STATIC_INSPECTION_B = 6
    AMI_AUTONOMOUS_DEMO = 7
    
    WHEEL_CIRCUMFERENCE = 1.617
    
    def __init__(self):
        super().__init__('autonomous_demo_state_wait')
        
        # Publishers
        self.ackermann_publisher = self.create_publisher(AckermannDrive, '/ackermann_cmd_controller', 1)
        self.brake_publisher = self.create_publisher(Bool, '/brake', 1)
        self.emergency_brake_publisher = self.create_publisher(Bool, '/emergency_brake', 1)
        self.chequered_flag_publisher = self.create_publisher(Bool, '/chequered_flag', 1)
        
        # Subscriber
        self.vcu2ai_subscriber = self.create_subscription(
            VCU2AI, 
            '/vcu2ai', 
            self.vcu2ai_callback, 
            1
        )
        
        # State variables from VCU2AI
        self.as_state = None
        self.ami_state = None
        self.rl_wheel_speed_rpm = None
        self.rr_wheel_speed_rpm = None
        self.rr_pulse_count = None
        self.rl_pulse_count = None
        
        # Control state
        self.demo_started = False
        self.demo_complete = False
        self.conditions_met = False
        
        self.get_logger().info("AutonomousDemoStateWait node initialized")
        self.get_logger().info("Waiting for AS_DRIVING and AMI_AUTONOMOUS_DEMO states...")

    def vcu2ai_callback(self, msg):
        self.as_state = msg.as_state
        self.ami_state = msg.ami_state
        self.rl_wheel_speed_rpm = msg.rl_wheel_speed_rpm
        self.rr_wheel_speed_rpm = msg.rr_wheel_speed_rpm
        self.rr_pulse_count = msg.rr_pulse_count
        self.rl_pulse_count = msg.rl_pulse_count
        
        # Check if both conditions are met
        if (self.as_state == self.AS_DRIVING and 
            self.ami_state == self.AMI_AUTONOMOUS_DEMO and 
            not self.demo_started):
            
            if not self.conditions_met:
                self.conditions_met = True
                self.get_logger().info("Both conditions met: AS_DRIVING and AMI_AUTONOMOUS_DEMO")
                self.get_logger().info("Starting autonomous demonstration...")
                self.start_demo()
        
        # Log state changes for debugging
        if hasattr(self, '_last_as_state') and self._last_as_state != self.as_state:
            self.get_logger().info(f"AS State changed to: {self.as_state}")
        if hasattr(self, '_last_ami_state') and self._last_ami_state != self.ami_state:
            self.get_logger().info(f"AMI State changed to: {self.ami_state}")
            
        self._last_as_state = self.as_state
        self._last_ami_state = self.ami_state

    def start_demo(self):
        if self.demo_started:
            return
            
        self.demo_started = True
        self.get_logger().info("Starting autonomous demonstration sequence")
        
        try:
            # Execute the demonstration sequence
            time.sleep(5)
            self.sweep_steering()
            time.sleep(1)
            self.accelerate_10m()
            time.sleep(2.5)  # Small delay between phases
            self.brake_10m()
            time.sleep(2.5)  # Small delay between phases
            self.accelerate_10m()
            time.sleep(2.5)  # Small delay before emergency brake
            self.emergency_brake()
            self.signal_completion()
            self.demo_complete = True
            
        except Exception as e:
            self.get_logger().error(f"Error during demonstration: {str(e)}")
            self.emergency_brake()

    def sweep_steering(self):
        self.get_logger().info("Starting steering sweep")
        angles = [-0.7, 0.7, 0.0]
        start_time = time.time()
        
        while rclpy.ok() and not self.demo_complete:
            elapsed_time = time.time() - start_time
            angle = 0.0
            
            if elapsed_time < 2:
                angle = self.linear_interpolate(elapsed_time, 0, 2, 0, angles[0])
            elif 2 <= elapsed_time <= 3:
                angle = angles[0]  # hold at -0.7
            elif 3 < elapsed_time < 5:
                angle = self.linear_interpolate(elapsed_time, 3, 5, angles[0], angles[1])
            elif 5 <= elapsed_time <= 6:
                angle = angles[1]  # hold at 0.7
            elif 6 < elapsed_time < 8.5:
                angle = self.linear_interpolate(elapsed_time, 6, 8.5, angles[1], angles[2])
            else:
                angle = angles[2]  # hold at 0

            
            if elapsed_time > 9:
                break

            ackermann_message = AckermannDrive()
            ackermann_message.steering_angle = angle
            self.ackermann_publisher.publish(ackermann_message)
            self.get_logger().info(f"Set steering angle to {angle:.3f}")
            
            time.sleep(0.1)
            
        self.get_logger().info("Steering sweep completed")

    def accelerate_10m(self):
        self.get_logger().info("Starting acceleration phase")
        
        start_stopwatch = time.time()
        ACCELERATION = 1.1  # m/s^2
        
        # Check if pulse count data is available and working
        if (self.rl_pulse_count is None or self.rr_pulse_count is None or 
            (self.rl_pulse_count == 0 and self.rr_pulse_count == 0)):
            self.get_logger().warn("Pulse count data not available or not working, using time-based acceleration")
            self.accelerate_time_based()
            return
            
        initial_rl = self.rl_pulse_count
        initial_rr = self.rr_pulse_count
        pulse_count_10m = 6 / self.WHEEL_CIRCUMFERENCE * 20  # 6m target distance
        
        self.get_logger().info(f"Starting acceleration with pulse counts - Initial RL: {initial_rl}, RR: {initial_rr}")
        self.get_logger().info(f"Target pulse count for 10m: {pulse_count_10m}")

        # Track if pulse counts are actually changing
        pulse_count_changed = False
        last_rl = initial_rl
        last_rr = initial_rr

        while rclpy.ok() and not self.demo_complete:
            elapsed = time.time() - start_stopwatch
            
            # Check distance traveled via pulse counts
            rl_distance_traveled = (self.rl_pulse_count - initial_rl) if self.rl_pulse_count is not None else 0
            rr_distance_traveled = (self.rr_pulse_count - initial_rr) if self.rr_pulse_count is not None else 0
            
            # Check if pulse counts are actually changing
            if (self.rl_pulse_count != last_rl or self.rr_pulse_count != last_rr):
                pulse_count_changed = True
                last_rl = self.rl_pulse_count
                last_rr = self.rr_pulse_count
            
            # Use the average of both wheels
            avg_distance = (rl_distance_traveled + rr_distance_traveled) / 2
            
            self.get_logger().info(f"Distance traveled - RL: {rl_distance_traveled:.1f}, RR: {rr_distance_traveled:.1f}, Avg: {avg_distance:.1f} pulses")
            
            # If pulse counts haven't changed after 2 seconds, fall back to time-based
            if elapsed > 2.0 and not pulse_count_changed:
                self.get_logger().warn("Pulse counts not changing, falling back to time-based acceleration")
                # Continue with time-based approach for remaining time
                remaining_time = 3.0  # 3 more seconds
                time_start = time.time()
                
                while rclpy.ok() and not self.demo_complete:
                    time_elapsed = time.time() - time_start
                    if time_elapsed > remaining_time:
                        break
                    
                    ackermann_message = AckermannDrive()
                    total_elapsed = elapsed + time_elapsed
                    ackermann_message.speed = total_elapsed * ACCELERATION
                    
                    self.get_logger().info(f"Time-based speed: {ackermann_message.speed:.2f} m/s")
                    self.ackermann_publisher.publish(ackermann_message)
                    
                    time.sleep(0.1)
                break
            
            # Stop if we've traveled enough distance OR if timeout reached
            if avg_distance >= pulse_count_10m or elapsed > 20:
                if elapsed > 20:
                    self.get_logger().warn("Acceleration phase timeout reached")
                else:
                    self.get_logger().info(f"Target distance reached: {avg_distance:.1f} >= {pulse_count_10m:.1f}")
                break
            
            ackermann_message = AckermannDrive()
            ackermann_message.speed = elapsed * ACCELERATION
            
            self.get_logger().info(f"Target speed: {ackermann_message.speed:.2f} m/s, "
                                 f"Wheel speeds: RL={self.rl_wheel_speed_rpm:.1f}, RR={self.rr_wheel_speed_rpm:.1f} rpm")
            
            self.ackermann_publisher.publish(ackermann_message)
            
            time.sleep(0.1)
                
        self.get_logger().info("Acceleration phase completed")
    
    def accelerate_time_based(self):
        """Fallback acceleration method when pulse count is not available"""
        start_time = time.time()
        ACCELERATION = 1.1  # m/s^2
        duration = 2.5  # seconds
        
        self.get_logger().info(f"Using time-based acceleration for {duration} seconds")
        
        while rclpy.ok() and not self.demo_complete:
            elapsed = time.time() - start_time
            if elapsed > duration:
                break
                
            ackermann_message = AckermannDrive()
            ackermann_message.speed = elapsed * ACCELERATION
            self.ackermann_publisher.publish(ackermann_message)
            
            self.get_logger().info(f"Time-based acceleration: {ackermann_message.speed:.2f} m/s")
            
            time.sleep(0.1)
    
    def brake_10m(self):
        self.get_logger().info("Starting braking phase")
        
        # First, stop sending speed commands
        ackermann_message = AckermannDrive()
        ackermann_message.speed = 0.0
        self.ackermann_publisher.publish(ackermann_message)
        
        self.get_logger().info("Speed command set to 0.0 m/s")
        
        # Apply brakes until speed is low enough
        brake_start_time = time.time()
        brake_applied = False
        
        while rclpy.ok() and not self.demo_complete:
            elapsed_brake_time = time.time() - brake_start_time
            
            # Check if we have valid wheel speed data
            if self.rl_wheel_speed_rpm is not None and self.rr_wheel_speed_rpm is not None:
                avg_wheel_speed = (self.rl_wheel_speed_rpm + self.rr_wheel_speed_rpm) / 2
                
                if avg_wheel_speed > 5:  # Still moving
                    brake_msg = Bool()
                    brake_msg.data = True
                    self.brake_publisher.publish(brake_msg)
                    brake_applied = True
                    self.get_logger().info(f"Braking - avg wheel speed: {avg_wheel_speed:.1f} rpm")
                else:
                    self.get_logger().info(f"Vehicle stopped - avg wheel speed: {avg_wheel_speed:.1f} rpm")
                    break
            else:
                # Fallback: brake for a fixed time if no wheel speed data
                if elapsed_brake_time < 12.0:  # Brake for 3 seconds
                    brake_msg = Bool()
                    brake_msg.data = True
                    self.brake_publisher.publish(brake_msg)
                    brake_applied = True
                    self.get_logger().info(f"Time-based braking: {elapsed_brake_time:.1f}s")
                else:
                    self.get_logger().info("Time-based braking completed")
                    break
            
            # Safety timeout
            if elapsed_brake_time > 20:
                self.get_logger().warn("Braking phase timeout reached")
                break
                
            time.sleep(0.1)
        
        # Always release brakes at the end
        if brake_applied:
            brake_msg = Bool()
            brake_msg.data = False
            self.brake_publisher.publish(brake_msg)
            self.get_logger().info("Brakes released")
        
        self.get_logger().info("Braking phase completed")

    def emergency_brake(self):
        self.get_logger().info("Activating emergency brake")
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_brake_publisher.publish(emergency_msg)
        
        # Also stop drivetrain
        ackermann_message = AckermannDrive()
        ackermann_message.speed = 0.0
        self.ackermann_publisher.publish(ackermann_message)

    def signal_completion(self):
        self.get_logger().info("Demonstration completed successfully")
        chequered_flag_msg = Bool()
        chequered_flag_msg.data = True
        self.chequered_flag_publisher.publish(chequered_flag_msg)

    @staticmethod
    def linear_interpolate(value, a1, a2, b1, b2):
        if value < a1:
            return b1
        elif value > a2:
            return b2
        else:
            left_span = a2 - a1
            right_span = b2 - b1
            value_scaled = float(value - a1) / float(left_span)
            return b1 + (value_scaled * right_span)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo_node = AutonomousDemoStateWait()
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()