"""Planar geometry used by the planners (no ROS imports)."""
import math

import numpy as np


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def to_local(points, x, y, yaw):
    """(N, 2) points from a world frame (map or odom) into the frame of the pose (x, y, yaw) given
    in that world frame: +x forward, +y left. Same rotation as fastslam_node._process_cones."""
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    dx = pts[:, 0] - x
    dy = pts[:, 1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


def to_world(points, x, y, yaw):
    """Inverse of to_local: (N, 2) points in the frame of the pose (x, y, yaw) into the world frame
    (map or odom) the pose is given in."""
    if len(points) == 0:
        return np.empty((0, 2))
    pts = np.asarray(points, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + pts[:, 0] * c - pts[:, 1] * s, y + pts[:, 0] * s + pts[:, 1] * c))
