#!/usr/bin/env python3
"""Ends each dynamic event the way the ADS-DV does: publish /chequered_flag, and the VCU
(fsai_api/ackermann_can on the car, ads_dv_vcu_sim in Gazebo) brakes to a standstill.

  acceleration  finish line crossed once          (/laps >= 1)
  autocross     one lap                           (/laps >= 1)
  trackdrive    ten laps                          (/laps >= laps_trackdrive)
  skidpad       the skidpad path has been driven  (/mission/path_complete from the pathfinder)

Publishes /mission_state (String): DRIVING, FINISHING (braking) or FINISHED (stopped).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt16


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.event = self.declare_parameter('event', 'trackdrive').value
        laps_trackdrive = int(self.declare_parameter('laps_trackdrive', 10).value)
        self.laps_needed = {'acceleration': 1, 'autocross': 1, 'trackdrive': laps_trackdrive,
                            'mppi_track': laps_trackdrive}.get(self.event)

        self.flag_pub = self.create_publisher(Bool, '/chequered_flag', 1)
        self.state_pub = self.create_publisher(String, '/mission_state', 1)
        self.create_subscription(UInt16, '/laps', self.laps_cb, 10)
        self.create_subscription(Bool, '/mission/path_complete', self.path_complete_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.2, self.tick)

        self.state = 'DRIVING'
        self.speed = 0.0
        self.get_logger().info(
            f"event {self.event}: finish after "
            + (f"{self.laps_needed} lap(s)" if self.laps_needed else "the skidpad path"))

    def laps_cb(self, msg):
        if self.laps_needed is not None and msg.data >= self.laps_needed:
            self.finish(f'{msg.data} lap(s) done')

    def path_complete_cb(self, msg):
        if msg.data and self.event == 'skidpad':
            self.finish('skidpad path driven')

    def odom_cb(self, msg):
        self.speed = abs(msg.twist.twist.linear.x)

    def finish(self, why):
        if self.state == 'DRIVING':
            self.state = 'FINISHING'
            self.get_logger().warn(f'{why}: chequered flag, braking to a stop')

    def tick(self):
        if self.state != 'DRIVING':
            self.flag_pub.publish(Bool(data=True))  # repeated: a late subscriber still gets it
            if self.state == 'FINISHING' and self.speed < 0.05:
                self.state = 'FINISHED'
                self.get_logger().info('mission finished: car stopped')
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
