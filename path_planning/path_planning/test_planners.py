"""Planner logic without a node or a simulator."""
import math

import numpy as np

from path_planning.centreline import CentrelineAlgorithm
from path_planning.geometry import to_local, to_world
from path_planning.remember_lap import RememberedLap
from path_planning.skidpad import SkidpadPlanner


class Log:
    def info(self, *a, **k):
        pass
    warn = debug = error = info


def straight_track(length=20.0, width=3.0, spacing=4.0):
    x = np.arange(2.0, length, spacing)
    return np.column_stack((x, np.full_like(x, -width / 2))), np.column_stack((x, np.full_like(x, width / 2)))


def test_transforms_round_trip():
    pts = np.random.default_rng(0).normal(0, 10, (10, 2))
    assert np.allclose(to_local(to_world(pts, 1.0, 2.0, 0.6), 1.0, 2.0, 0.6), pts)


def test_centreline_on_a_straight_is_the_middle():
    yellow, blue = straight_track()
    path = CentrelineAlgorithm(Log()).calculate_path(yellow.tolist(), blue.tolist())
    assert path is not None and len(path) >= 3
    assert np.all(np.abs(path[:, 1]) < 0.05) and np.all(np.diff(path[:, 0]) > 0)


def test_one_boundary_waypoints_stay_off_the_cones():
    # only the left (blue) side of a straight is mapped beyond 6 m: waypoints are placed from it alone
    yellow, blue = straight_track()
    yellow = yellow[yellow[:, 0] < 6.0]
    far_yellow = np.array([[2.0, -9.0]])
    path = CentrelineAlgorithm(Log()).calculate_path(np.vstack([yellow, far_yellow]).tolist(), blue.tolist())
    assert path is not None
    d = np.linalg.norm(path[:, None, :] - blue[None, :, :], axis=2).min()
    assert d > 0.8


def circle(radius=10.0, n=126):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.column_stack((radius * np.sin(a), radius * (1 - np.cos(a))))   # starts at the origin heading +x


def drive_lap(lap, pts):
    for k, (x, y) in enumerate(pts):
        lap.record(x, y, laps=0)
    for x, y in pts[:3]:
        lap.record(x, y, laps=1)


def test_remembered_lap_is_followed_and_dropped_near_a_cone():
    lap = RememberedLap(Log())
    assert lap.path(0.0, 0.0, 0.0, []) is None          # nothing stored yet
    pts = circle()
    drive_lap(lap, pts)
    assert lap.loop is not None
    path = lap.path(0.0, 0.0, 0.0, [np.empty((0, 2))])
    assert path is not None and path[0].tolist() == [0.0, 0.0]
    assert np.hypot(*np.diff(path, axis=0).T).sum() > 15.0   # about 20 m of the stored line ahead
    cone_on_line = to_local(pts[10:11], 0.0, 0.0, 0.0)
    assert lap.path(0.0, 0.0, 0.0, [cone_on_line]) is None
    assert lap.path(0.0, -5.0, 0.0, []) is None         # car too far from the stored line


def test_skidpad_path_moves_forward():
    sp = SkidpadPlanner(Log())
    window, done = sp.plan(np.array([-10.0, 0.0]))
    assert len(window) > 2 and not done
    assert window[-1][0] > window[0][0]
