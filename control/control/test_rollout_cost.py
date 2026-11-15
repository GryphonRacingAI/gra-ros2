"""rollout_cost() without a node: the weights come from a plain namespace."""
from types import SimpleNamespace

import numpy as np

from control.mppi_ros_modified import rollout_cost


def weights(**over):
    c = SimpleNamespace(
        rear_to_base=0.767, path_spacing=0.5, p={'wheelbase': 1.53}, a_lat_max=8.0,
        w_path=10.0, w_heading=5.0, w_progress=1.0, w_speed=1.0, w_overspeed=5.0, w_grip=1.0,
        w_steer_rate=1.0, w_accel=0.1, w_obstacle=500.0, w_collision=3000.0, obstacle_margin=0.2,
        obstacle_window=4.0, circle_offsets=np.array([-0.5, 0.0, 0.767, 1.534, 1.95]),
        circle_radii=np.array([0.35, 0.67, 0.40, 0.58, 0.30]))
    c.__dict__.update(over)
    return c


def straight_rollouts(offsets, H=20, v=5.0, dt=0.05):
    """One rollout per lateral offset: driving straight along +x at constant speed."""
    K = len(offsets)
    P = np.zeros((K, H, 5))
    P[:, :, 0] = v * dt * np.arange(1, H + 1)
    P[:, :, 1] = np.asarray(offsets)[:, None]
    P[:, :, 3] = v
    return P, np.zeros((K, H, 2))


REF = np.column_stack((np.arange(0.0, 15.0, 0.5), np.zeros(30)))
REF_HEADING = np.zeros(30)
REF_V = np.full(30, 5.0)


def test_on_path_is_cheapest():
    P, U = straight_rollouts([0.0, 0.5, -1.0])
    cost, parts = rollout_cost(P, U, 0.0, REF, REF_HEADING, REF_V, np.empty((0, 3)), weights())
    assert parts is None
    assert cost[0] < cost[1] < cost[2]


def test_cone_on_the_path_costs_more_than_a_cone_beside_it():
    P, U = straight_rollouts([0.0])
    on_path = rollout_cost(P, U, 0.0, REF, REF_HEADING, REF_V, np.array([[4.0, 0.0, 0.16]]), weights())[0]
    beside = rollout_cost(P, U, 0.0, REF, REF_HEADING, REF_V, np.array([[4.0, 2.5, 0.16]]), weights())[0]
    free = rollout_cost(P, U, 0.0, REF, REF_HEADING, REF_V, np.empty((0, 3)), weights())[0]
    assert on_path[0] > beside[0]
    assert np.isclose(beside[0], free[0])


def test_debug_breakdown():
    P, U = straight_rollouts([0.0, 0.3])
    cost, parts = rollout_cost(P, U, 0.0, REF, REF_HEADING, REF_V, np.array([[4.0, 0.9, 0.16]]), weights(), debug=True)
    assert set(parts) == {'path', 'progress', 'speed', 'obst'}
    assert all(v.shape == cost.shape for v in parts.values())
