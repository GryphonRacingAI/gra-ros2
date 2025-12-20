"""
Mission Supervisor Node for FSAI

- Implements state machine for autocross (right circle x2, left circle x2, exit)
- Path planning: multiple best path method
- State-based path selection (like skidpad)

Author: @musbahi-git
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        self.declare_parameter('event', 'autocross')
        self.event = self.get_parameter('event').value
        self.state = 'INIT'
        self.lap_count = 0
        self.circle = 'right'  # Start with right circle
        self.path_pub = self.create_publisher(Path, '/mission_path', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info(f"MissionSupervisor started for event: {self.event}")

    def timer_cb(self):
        # State machine for autocross/skidpad: right x2, left x2, exit
        if self.event in ['autocross', 'skidpad']:
            if self.state == 'INIT':
                self.state = 'RIGHT_CIRCLE'
            elif self.state == 'RIGHT_CIRCLE':
                # Simulate lap completion (replace with real detection)
                if self.lap_count < 2:
                    self.publish_path('right')
                else:
                    self.state = 'LEFT_CIRCLE'
            elif self.state == 'LEFT_CIRCLE':
                if self.lap_count < 2:
                    self.publish_path('left')
                else:
                    self.state = 'EXIT'
            elif self.state == 'EXIT':
                self.publish_path('exit')
        # Publish state
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_path(self, which):
        # Multiple best path method: generate several candidate paths, pick best for state
        path = Path()
        path.header.frame_id = 'map'
        t = np.linspace(0, 2*np.pi, 50)
        if which == 'right':
            # Right circle (center at +y)
            cx, cy, r = 5.0, -5.0, 5.0
            for theta in t:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = cx + r * np.cos(theta)
                pose.pose.position.y = cy + r * np.sin(theta)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        elif which == 'left':
            # Left circle (center at -y)
            cx, cy, r = 5.0, 5.0, 5.0
            for theta in t:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = cx + r * np.cos(-theta)
                pose.pose.position.y = cy + r * np.sin(-theta)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        elif which == 'exit':
            # Exit path: straight line from center
            for i in range(50):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = 5.0 + i * 0.2
                pose.pose.position.y = 0.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_pub.publish(path)
        self.get_logger().info(f"Published {which} path for state {self.state}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
