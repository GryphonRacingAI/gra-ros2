#!/usr/bin/env python3
"""
Local-frame pure pursuit controller.

Expects /path in a body-relative frame (track_pathfinder uses velodyne:
car at origin, +x forward). Odometry is only used for measured speed;
pose from odom is ignored when use_local_frame is true (default).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from control.path_processor import PathProcessor, PathProcessorConfig


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.path_topic = self.declare_parameter('path_topic', '/path').value
        self.use_local_frame = self.declare_parameter('use_local_frame', True).value
        self.expected_path_frame = self.declare_parameter('path_frame', 'velodyne').value

        self.dt = self.declare_parameter('dt', 0.05).value
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 3.0).value
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.max_steer = self.declare_parameter('max_steer', math.pi / 6).value
        self.min_speed = self.declare_parameter('min_speed', 1.0).value
        self.max_speed = self.declare_parameter('max_speed', 6.0).value
        self.target_speed = self.declare_parameter('target_speed', 3.0).value
        self.use_curvature_speed = self.declare_parameter('use_curvature_speed', True).value
        self.a_lat_max = self.declare_parameter('a_lat_max', 2.0).value
        self.v_max_straight = self.declare_parameter('v_max_straight', 7.0).value
        self.v_min = self.declare_parameter('v_min', 1.0).value

        self.path_processor = PathProcessor(PathProcessorConfig(
            a_lat_max=self.a_lat_max,
            v_max_straight=self.v_max_straight,
            v_min=self.v_min,
        ))

        self.speed_mps = 0.0
        self.have_odom = False
        self.path_arr = None
        self.path_v_ref = None
        self._path_frame_warned = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pub_ackermann = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        self.pub_viz = self.create_publisher(Path, '/viz/pure_pursuit_lookahead', 10)

        self.create_timer(self.dt, self.control_loop)

        mode = "local/velodyne" if self.use_local_frame else "odom-global (unsupported for PP pose)"
        self.get_logger().info(
            f"PurePursuit initialized ({mode}), lookahead={self.lookahead_distance}m, "
            f"path_topic={self.path_topic}")

    def odom_callback(self, msg):
        self.speed_mps = float(msg.twist.twist.linear.x)
        self.have_odom = True

    def path_callback(self, msg):
        n = len(msg.poses)
        if n < 2:
            return

        frame = msg.header.frame_id or ''
        if self.use_local_frame and not self._path_frame_warned:
            if frame and frame not in (self.expected_path_frame, 'base_link', 'velodyne', ''):
                self.get_logger().warn(
                    f"use_local_frame=true but /path frame_id='{frame}' "
                    f"(expected '{self.expected_path_frame}' or body-local).")
            self._path_frame_warned = True

        pts = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses],
            dtype=float)

        if self.use_curvature_speed:
            try:
                processed = self.path_processor(pts)
                self.path_arr = processed.points
                self.path_v_ref = processed.speed_ref
            except ValueError:
                self.path_arr = pts
                self.path_v_ref = None
        else:
            self.path_arr = pts
            self.path_v_ref = None

        self.get_logger().info(
            f"Path received with {n} points (frame='{frame or 'unset'}').",
            throttle_duration_sec=2.0)

    def find_lookahead(self, path):
        """Return (x, y, index) of the lookahead point in the vehicle frame."""
        Ld = self.lookahead_distance
        dists = np.linalg.norm(path, axis=1)

        # Prefer the first point at least Ld ahead of the car (origin)
        for i, d in enumerate(dists):
            if d >= Ld and path[i, 0] > 0.0:  # prefer points in front
                return path[i, 0], path[i, 1], i

        # Fallback: farthest point with positive x, else last point
        front = np.where(path[:, 0] > 0.1)[0]
        if front.size > 0:
            i = int(front[np.argmax(dists[front])])
            return path[i, 0], path[i, 1], i

        i = len(path) - 1
        return path[i, 0], path[i, 1], i

    def control_loop(self):
        if not self.have_odom:
            self.get_logger().info("Waiting for odom (speed)", throttle_duration_sec=2.0)
            return
        if self.path_arr is None or len(self.path_arr) < 2:
            self.get_logger().info("Waiting for path", throttle_duration_sec=2.0)
            return

        if not self.use_local_frame:
            self.get_logger().error(
                "pure_pursuit only supports use_local_frame=true (body-relative /path)",
                throttle_duration_sec=5.0)
            return

        path = self.path_arr
        lx, ly, idx = self.find_lookahead(path)
        Ld = math.hypot(lx, ly)
        if Ld < 1e-3:
            steer = 0.0
        else:
            # α = angle to lookahead; δ = atan2(2 L sin(α), Ld)
            alpha = math.atan2(ly, lx)
            steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
            steer = clamp(steer, -self.max_steer, self.max_steer)

        if self.path_v_ref is not None and 0 <= idx < len(self.path_v_ref):
            speed = float(self.path_v_ref[idx])
        else:
            speed = float(self.target_speed)
        speed = clamp(speed, self.min_speed, self.max_speed)

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'
        drive.drive.steering_angle = float(steer)
        drive.drive.speed = float(speed)
        self.pub_drive.publish(drive)

        ack = AckermannDrive()
        ack.steering_angle = float(steer)
        ack.speed = float(speed)
        self.pub_ackermann.publish(ack)

        self._publish_lookahead_viz(lx, ly)

    def _publish_lookahead_viz(self, lx, ly):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.expected_path_frame
        for x, y in ((0.0, 0.0), (lx, ly)):
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_viz.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
