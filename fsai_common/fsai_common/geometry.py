"""Planar geometry shared by the FSAI packages (no ROS imports: usable in plain tests)."""
import math

import numpy as np


def yaw_from_quaternion(q):
    """Yaw (rad) of a quaternion given as a message (x, y, z, w attributes) or an (x, y, z, w) sequence."""
    x, y, z, w = (q.x, q.y, q.z, q.w) if hasattr(q, 'w') else q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def to_world(points, x, y, yaw):
    """(N, 2) points in the frame of the pose (x, y, yaw) -> the world frame the pose is given in."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))


def to_local(points, x, y, yaw):
    """Inverse of to_world: (N, 2) world points -> the frame of the pose (x, y, yaw); +x forward, +y left."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))
