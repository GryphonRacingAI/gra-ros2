#!/usr/bin/env python3
"""Cones straight from the lidar, as obstacles for the MPPI controller (/lidar_cones, odom frame).

Why: the cone detector needs a YOLO box, so a cone is only detected while it is in the camera's
~70 degree view, and close cones at the edge of the image are often missed. The lidar clusters need
no box, so they give the controller every cone in the lidar's view whatever its colour, and the
short memory keeps the ones the car is passing. Note the simulated Velodyne only covers +-45 degrees
ahead (ads_dv.sdf), not all around: cones that never enter that sector (the inner cones of the
mppi_track hairpin) are not seen here either; the planner's one-boundary waypoints cover those.

Pipeline (per cloud): points between ground_z + min_height and ground_z + max_height (cone height
0.325 m; the ground is flat at ground_z in the lidar frame), between min_range and max_range and
outside the car's own footprint -> grid clustering (cell m, 8-neighbour) -> clusters no wider than
max_extent with at least min_points -> centre pushed back by cone_radius (the lidar sees the front
of a cone) -> base_link -> odom with the /odom pose. Colour is unknown, so this only feeds the
obstacle cost, never the planner.
"""
from collections import defaultdict

import numpy as np
import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

from control.geometry import to_world, yaw_from_quaternion


def cluster_cones(xy, cell=0.25, max_extent=0.5, min_points=2):
    """Grid clustering of (N, 2) points; returns the centres of the cone-sized clusters."""
    if len(xy) == 0:
        return np.empty((0, 2))
    keys = np.floor(xy / cell).astype(int)
    buckets = defaultdict(list)
    for i, k in enumerate(map(tuple, keys)):
        buckets[k].append(i)
    label = -np.ones(len(xy), dtype=int)
    centres = []
    for start in range(len(xy)):
        if label[start] >= 0:
            continue
        label[start] = start
        stack, members = [start], [start]
        while stack:
            kx, ky = keys[stack.pop()]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in buckets.get((kx + dx, ky + dy), ()):
                        if label[j] < 0:
                            label[j] = start
                            stack.append(j)
                            members.append(j)
        pts = xy[members]
        if len(pts) >= min_points and np.all(pts.max(axis=0) - pts.min(axis=0) <= max_extent):
            centres.append(pts.mean(axis=0))
    return np.array(centres).reshape(-1, 2)


class LidarConeObstacles(Node):
    def __init__(self):
        super().__init__('lidar_cone_obstacles')
        self.cloud_topic = self.declare_parameter('cloud_topic', '/velodyne_points').value
        self.lidar_x = self.declare_parameter('lidar_x', 0.52).value        # lidar ahead of base_link (m)
        self.ground_z = self.declare_parameter('ground_z', -0.60).value     # ground height in the lidar frame
        self.min_height = self.declare_parameter('min_height', 0.04).value
        self.max_height = self.declare_parameter('max_height', 0.40).value
        self.min_range = self.declare_parameter('min_range', 1.2).value
        self.max_range = self.declare_parameter('max_range', 10.0).value
        self.cell = self.declare_parameter('cell', 0.25).value
        self.max_extent = self.declare_parameter('max_extent', 0.5).value
        self.min_points = int(self.declare_parameter('min_points', 2).value)
        self.cone_radius = self.declare_parameter('cone_radius', 0.08).value
        # the car's own body in base_link (x from tail to nose, half width): never a cone
        self.body_x = self.declare_parameter('body_x', [-1.3, 1.3]).value
        self.body_half_width = self.declare_parameter('body_half_width', 0.8).value
        # Short-term memory in odom: a cone right beside the car (hairpin apex) is below the lidar's
        # view, so the cones seen in the last memory_time s stay obstacles. A cone must have been seen
        # in min_hits clouds (noise), detections within merge_dist of a remembered cone update it.
        self.memory_time = self.declare_parameter('memory_time', 3.0).value
        self.merge_dist = self.declare_parameter('merge_dist', 0.4).value
        self.min_hits = int(self.declare_parameter('min_hits', 2).value)
        self.memory = np.empty((0, 5))  # x, y, last seen (s), hits, unused

        self.n_clouds = 0
        self.odom = None
        self.odom_hist = []  # (stamp s, x, y, yaw): the cloud is placed with the pose at its own stamp
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '/lidar_cones', 1)
        self.get_logger().info(f"lidar cone obstacles from {self.cloud_topic} -> /lidar_cones (odom)")

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = (p.x, p.y, yaw_from_quaternion(q))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_hist.append((t,) + self.odom)
        if len(self.odom_hist) > 200:
            del self.odom_hist[:100]

    def pose_at(self, t):
        """/odom pose nearest in time to t (odom runs at >= 50 Hz, so within 10 ms)."""
        best = min(self.odom_hist, key=lambda h: abs(h[0] - t))
        return best[1:], abs(best[0] - t)

    def remember(self, world, now):
        self.memory = _remember(self.memory, world, now, self.memory_time, self.merge_dist)

    def cloud_cb(self, msg):
        if self.odom is None:
            return
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        z = pts[:, 2] - self.ground_z
        bx = pts[:, 0] + self.lidar_x                                   # base_link x
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = ((z > self.min_height) & (z < self.max_height) & (r > self.min_range) & (r < self.max_range)
                & ~((bx > self.body_x[0]) & (bx < self.body_x[1]) & (np.abs(pts[:, 1]) < self.body_half_width)))
        centres = cluster_cones(pts[keep, :2].astype(float), self.cell, self.max_extent, self.min_points)
        if len(centres):
            rng = np.linalg.norm(centres, axis=1, keepdims=True)
            centres = centres + centres / np.maximum(rng, 1e-6) * self.cone_radius
            centres[:, 0] += self.lidar_x                                # lidar -> base_link
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        (ox, oy, yaw), dt = self.pose_at(now)
        self.n_clouds += 1
        if self.n_clouds % 50 == 0:
            self.get_logger().info(f"clouds {self.n_clouds}: pose dt {dt * 1000:.0f} ms, "
                                   f"remembered {len(self.memory)}, published {int((self.memory[:, 3] >= self.min_hits).sum())}")
        if dt > 0.05:  # no pose for this cloud: do not place it
            return
        world = to_world(centres, ox, oy, yaw)
        self.remember(world, now)
        # forget cones far from the car (their old pose estimate is the least reliable)
        far = np.hypot(self.memory[:, 0] - ox, self.memory[:, 1] - oy) > self.max_range
        self.memory = self.memory[~far]
        good = self.memory[self.memory[:, 3] >= self.min_hits]
        xyz = np.zeros((len(good), 3), dtype=np.float32)
        xyz[:, :2] = good[:, :2]
        header = Header(stamp=msg.header.stamp, frame_id='odom')
        self.pub.publish(pc2.create_cloud_xyz32(header, xyz.tolist()))


def _remember(memory, world, now, memory_time, merge_dist):
    """Merge detections (N, 2) into memory rows [x, y, last_seen, hits, _]; drop stale rows."""
    memory = memory[now - memory[:, 2] <= memory_time]
    new = []
    for p in world:
        if len(memory):
            d = np.hypot(memory[:, 0] - p[0], memory[:, 1] - p[1])
            j = int(np.argmin(d))
            if d[j] < merge_dist:
                h = memory[j, 3]
                memory[j, :2] = (memory[j, :2] * min(h, 10) + p) / (min(h, 10) + 1)
                memory[j, 2], memory[j, 3] = now, h + 1
                continue
        new.append([p[0], p[1], now, 1, 0])
    if new:
        memory = np.vstack([memory, np.array(new)])
    return memory


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
