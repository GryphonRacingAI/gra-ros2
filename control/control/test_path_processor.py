import numpy as np
import pytest

from control.path_processor import (
    PathProcessor,
    PathProcessorConfig,
    ProcessedPath,
    _interpolate_points,
)


@pytest.fixture
def default_processor():
    return PathProcessor(PathProcessorConfig())


# ---------------------------------------------------------------------------
# Interpolation density
# ---------------------------------------------------------------------------
class TestInterpolatePoints:
    def test_output_length(self):
        """Output length should be (N-1) * (num_between+1) + 1 for N input points."""
        N = 5
        num_between = 4
        pts = np.column_stack([np.linspace(0, 1, N), np.zeros(N)])
        result = _interpolate_points(pts, num_between=num_between)
        expected_len = (N - 1) * (num_between + 1) + 1
        assert len(result) == expected_len

    def test_endpoints_preserved(self):
        pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        result = _interpolate_points(pts, num_between=3)
        np.testing.assert_array_almost_equal(result[0], pts[0])
        np.testing.assert_array_almost_equal(result[-1], pts[-1])


# ---------------------------------------------------------------------------
# Straight line
# ---------------------------------------------------------------------------
class TestStraightLine:
    def test_constant_heading(self, default_processor):
        pts = np.column_stack([np.linspace(0, 10, 20), np.zeros(20)])
        result = default_processor(pts)
        # Heading should be ~0 (pointing along +x) everywhere
        np.testing.assert_allclose(result.heading, 0.0, atol=1e-6)

    def test_speed_equals_v_max(self, default_processor):
        pts = np.column_stack([np.linspace(0, 10, 20), np.zeros(20)])
        result = default_processor(pts)
        # Zero curvature → v_ref = sqrt(a_lat_max / eps) clipped to v_max_straight
        np.testing.assert_allclose(
            result.speed_ref, default_processor._cfg.v_max_straight, atol=1e-2
        )


# ---------------------------------------------------------------------------
# Circle arc
# ---------------------------------------------------------------------------
class TestCircleArc:
    def test_curvature_and_speed(self):
        R = 5.0
        theta = np.linspace(0, np.pi, 500)
        pts = np.column_stack([R * np.cos(theta), R * np.sin(theta)])

        cfg = PathProcessorConfig(
            a_lat_max=2.0, v_max_straight=20.0, v_min=0.5, num_between=0
        )
        proc = PathProcessor(cfg)
        result = proc(pts)

        expected_speed = np.sqrt(cfg.a_lat_max * R)
        # Check interior points (edges have gradient boundary effects)
        n = len(result.speed_ref)
        interior = result.speed_ref[n // 4 : 3 * n // 4]
        np.testing.assert_allclose(interior, expected_speed, atol=0.5)


# ---------------------------------------------------------------------------
# S-curve
# ---------------------------------------------------------------------------
class TestSCurve:
    def test_heading_changes_sign(self, default_processor):
        t = np.linspace(0, 2 * np.pi, 100)
        pts = np.column_stack([t, np.sin(t)])
        result = default_processor(pts)

        # Heading should go positive then negative (or vice-versa)
        mid = len(result.heading) // 2
        first_half_mean = result.heading[:mid].mean()
        second_half_mean = result.heading[mid:].mean()
        assert np.sign(first_half_mean) != np.sign(second_half_mean)

    def test_speed_dips_at_curves(self):
        t = np.linspace(0, 2 * np.pi, 200)
        pts = np.column_stack([t, np.sin(t)])
        # Use a high v_max_straight so clipping doesn't mask the speed dip
        cfg = PathProcessorConfig(a_lat_max=2.0, v_max_straight=100.0, v_min=0.1, num_between=0)
        proc = PathProcessor(cfg)
        result = proc(pts)

        # The minimum speed (at max curvature) should be lower than the max speed
        assert result.speed_ref.min() < result.speed_ref.max()


# ---------------------------------------------------------------------------
# Minimum points (exactly 2)
# ---------------------------------------------------------------------------
class TestMinimumPoints:
    def test_two_points(self, default_processor):
        pts = np.array([[0.0, 0.0], [1.0, 0.0]])
        result = default_processor(pts)
        assert isinstance(result, ProcessedPath)
        assert result.points.shape[0] >= 2
        assert result.heading.shape == (result.points.shape[0],)
        assert result.speed_ref.shape == (result.points.shape[0],)


# ---------------------------------------------------------------------------
# Single point / empty
# ---------------------------------------------------------------------------
class TestInvalidInputs:
    def test_single_point_raises(self, default_processor):
        pts = np.array([[1.0, 2.0]])
        with pytest.raises(ValueError, match="At least 2 waypoints"):
            default_processor(pts)

    def test_empty_raises(self, default_processor):
        pts = np.empty((0, 2))
        with pytest.raises(ValueError, match="At least 2 waypoints"):
            default_processor(pts)

    def test_wrong_shape_raises(self, default_processor):
        pts = np.array([1.0, 2.0, 3.0])
        with pytest.raises(ValueError, match="must be an \\(N, 2\\) array"):
            default_processor(pts)


# ---------------------------------------------------------------------------
# Config edge cases
# ---------------------------------------------------------------------------
class TestConfigEdgeCases:
    def test_v_min_equals_v_max(self):
        cfg = PathProcessorConfig(a_lat_max=2.0, v_max_straight=3.0, v_min=3.0)
        proc = PathProcessor(cfg)
        pts = np.column_stack([np.linspace(0, 10, 20), np.zeros(20)])
        result = proc(pts)
        np.testing.assert_allclose(result.speed_ref, 3.0)

    def test_a_lat_max_zero(self):
        cfg = PathProcessorConfig(a_lat_max=0.0, v_max_straight=7.0, v_min=1.0)
        proc = PathProcessor(cfg)
        pts = np.column_stack([np.linspace(0, 10, 20), np.zeros(20)])
        result = proc(pts)
        # sqrt(0 / ...) = 0, clipped to v_min
        np.testing.assert_allclose(result.speed_ref, cfg.v_min)


# ---------------------------------------------------------------------------
# ProcessedPath shape consistency
# ---------------------------------------------------------------------------
class TestOutputShapes:
    def test_all_shapes_match(self, default_processor):
        pts = np.column_stack([np.linspace(0, 5, 10), np.linspace(0, 3, 10)])
        result = default_processor(pts)
        M = result.points.shape[0]
        assert result.points.shape == (M, 2)
        assert result.heading.shape == (M,)
        assert result.speed_ref.shape == (M,)
