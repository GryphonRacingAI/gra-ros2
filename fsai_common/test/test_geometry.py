import math

import numpy as np

from fsai_common.geometry import to_local, to_world, yaw_from_quaternion


def test_yaw_from_quaternion_tuple_and_message():
    yaw = 0.7
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

    class Msg:
        x, y, z, w = q

    assert math.isclose(yaw_from_quaternion(q), yaw)
    assert math.isclose(yaw_from_quaternion(Msg()), yaw)


def test_to_world_known_point():
    # car at (1, 2) facing +y: a point 3 m ahead and 1 m to the left is at (0, 5)
    assert np.allclose(to_world([[3.0, 1.0]], 1.0, 2.0, math.pi / 2), [[0.0, 5.0]])


def test_round_trip_and_empty():
    rng = np.random.default_rng(0)
    pts = rng.normal(0, 10, (20, 2))
    pose = (3.0, -4.0, 2.1)
    assert np.allclose(to_local(to_world(pts, *pose), *pose), pts)
    assert to_world(np.empty((0, 2)), *pose).shape == (0, 2)
    assert to_local([], *pose).shape == (0, 2)
