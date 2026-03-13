#!/usr/bin/env python3
import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from mppi_ros_modified import (
    interpolate_points, clamp, normalize_angle, dynamics_vec,
    DT, MAX_A, MAX_STEER, MIN_V, MAX_V, H,
)


class TestInterpolatePoints:
    def test_basic_shape(self):
        arr = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        result = interpolate_points(arr, num_between=3)
        expected_len = 1 + (len(arr) - 1) * (3 + 1)
        assert len(result) == expected_len

    def test_preserves_endpoints(self):
        arr = np.array([[0.0, 0.0], [10.0, 5.0]])
        result = interpolate_points(arr, num_between=4)
        np.testing.assert_array_almost_equal(result[0], arr[0])
        np.testing.assert_array_almost_equal(result[-1], arr[-1])

    def test_single_segment(self):
        arr = np.array([[0.0, 0.0], [1.0, 1.0]])
        result = interpolate_points(arr, num_between=3)
        assert len(result) == 3 + 2

    def test_monotonic_x(self):
        arr = np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]])
        result = interpolate_points(arr, num_between=2)
        diffs = np.diff(result[:, 0])
        assert np.all(diffs >= 0)

    def test_no_duplicate_junction_points(self):
        arr = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        result = interpolate_points(arr, num_between=0)
        assert len(result) == 3
        np.testing.assert_array_almost_equal(result[1], [1.0, 0.0])


class TestClamp:
    def test_within_range(self):
        x = np.array([1.0, 2.0, 3.0])
        result = clamp(x, 0.0, 5.0)
        np.testing.assert_array_equal(result, x)

    def test_clamp_low(self):
        x = np.array([-5.0, -1.0])
        result = clamp(x, 0.0, 10.0)
        np.testing.assert_array_equal(result, [0.0, 0.0])

    def test_clamp_high(self):
        x = np.array([15.0, 20.0])
        result = clamp(x, 0.0, 10.0)
        np.testing.assert_array_equal(result, [10.0, 10.0])

    def test_scalar_like(self):
        result = clamp(np.array(5.0), 0.0, 3.0)
        assert float(result) == 3.0


class TestNormalizeAngle:
    def test_within_pi(self):
        assert normalize_angle(0.5) == pytest.approx(0.5, abs=1e-10)

    def test_wrap_positive(self):
        result = normalize_angle(3 * np.pi)
        assert -np.pi <= result <= np.pi
        assert result == pytest.approx(np.pi, abs=1e-10)

    def test_wrap_negative(self):
        result = normalize_angle(-3 * np.pi)
        assert -np.pi <= result <= np.pi

    def test_exact_pi(self):
        result = normalize_angle(np.pi)
        assert abs(result) == pytest.approx(np.pi, abs=1e-10)

    def test_zero(self):
        assert normalize_angle(0.0) == pytest.approx(0.0, abs=1e-10)


class TestDynamicsVec:
    def test_straight_line_zero_steer(self):
        k, h = 10, 5
        X = np.zeros((k, h + 1, 4))
        X[:, 0, 3] = 2.0
        U = np.zeros((k, h, 2))
        U[:, :, 0] = 0.0

        result = dynamics_vec(X, U)
        for step in range(1, h + 1):
            np.testing.assert_allclose(
                result[:, step, 0], 2.0 * DT * step, atol=1e-10
            )
            np.testing.assert_allclose(result[:, step, 1], 0.0, atol=1e-10)
            np.testing.assert_allclose(result[:, step, 3], 2.0, atol=1e-10)

    def test_speed_clamp_max(self):
        k, h = 5, 20
        X = np.zeros((k, h + 1, 4))
        X[:, 0, 3] = MAX_V - 0.1
        U = np.zeros((k, h, 2))
        U[:, :, 0] = MAX_A

        result = dynamics_vec(X, U)
        assert np.all(result[:, :, 3] <= MAX_V + 1e-10)

    def test_speed_clamp_min(self):
        k, h = 5, 20
        X = np.zeros((k, h + 1, 4))
        X[:, 0, 3] = MIN_V + 0.1
        U = np.zeros((k, h, 2))
        U[:, :, 0] = -MAX_A

        result = dynamics_vec(X, U)
        assert np.all(result[:, :, 3] >= MIN_V - 1e-10)

    def test_output_shape(self):
        k, h = 3, 4
        X = np.zeros((k, h + 1, 4))
        U = np.zeros((k, h, 2))
        result = dynamics_vec(X, U)
        assert result.shape == (k, h + 1, 4)

    def test_constant_acceleration(self):
        k = 1
        h = 5
        X = np.zeros((k, h + 1, 4))
        X[:, 0, 3] = 0.0
        U = np.zeros((k, h, 2))
        U[:, :, 0] = 1.0

        result = dynamics_vec(X, U)
        for step in range(1, h + 1):
            expected_v = min(step * DT * 1.0, MAX_V)
            assert result[0, step, 3] == pytest.approx(expected_v, abs=1e-10)
