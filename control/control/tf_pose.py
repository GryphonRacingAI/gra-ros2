"""Planar pose of one TF frame in another at a given time (tf2, interpolated)."""
from rclpy.time import Time
from tf2_ros import TransformException

from control.geometry import yaw_from_quaternion


def pose2d_at(tf_buffer, stamp, target='odom', source='base_link', tolerance=0.05):
    """(x, y, yaw) of `source` in `target` at `stamp` (a builtin_interfaces Time message), or None.

    tf2 interpolates between the samples around `stamp`. A stamp newer than the last sample cannot
    be interpolated; the last sample is used then if it is within `tolerance` s of `stamp`.
    """
    try:
        t = tf_buffer.lookup_transform(target, source, Time.from_msg(stamp))
    except TransformException:
        try:
            t = tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        age = (Time.from_msg(stamp) - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if abs(age) > tolerance:
            return None
    tr = t.transform
    return tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation)
