"""Tests for the drone_sim.geofence_utils module."""

from drone_sim.geofence_utils import check_boundary
import pytest

BOUNDS_MIN = (-10.0, -10.0, 0.0)
BOUNDS_MAX = (10.0, 10.0, 20.0)
MARGIN = 1.0


def test_center_position_is_safe():
    """A position well inside the bounds and margin is safe and unclamped."""
    status, clamped = check_boundary(0.0, 0.0, 5.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'safe'
    assert clamped == pytest.approx((0.0, 0.0, 5.0))


def test_position_inside_margin_is_warning():
    """A position between the hard bound and the safety margin is a warning."""
    status, clamped = check_boundary(9.5, 0.0, 5.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'warning'
    assert clamped == pytest.approx((9.0, 0.0, 5.0))


def test_position_outside_max_bound_is_breach():
    """A position at or beyond the upper bound is a breach."""
    status, clamped = check_boundary(10.0, 0.0, 5.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'
    assert clamped == pytest.approx((9.0, 0.0, 5.0))


def test_position_below_min_bound_is_breach():
    """A position below the lower bound on any axis is a breach."""
    status, _ = check_boundary(-10.5, 0.0, 5.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'


def test_clamped_position_respects_safe_bounds():
    """The clamped position always lies within the margin-shrunk safe bounds."""
    _, clamped = check_boundary(100.0, -100.0, 50.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    safe_min = (BOUNDS_MIN[0] + MARGIN, BOUNDS_MIN[1] + MARGIN, BOUNDS_MIN[2] + MARGIN)
    safe_max = (BOUNDS_MAX[0] - MARGIN, BOUNDS_MAX[1] - MARGIN, BOUNDS_MAX[2] - MARGIN)
    for value, lo, hi in zip(clamped, safe_min, safe_max):
        assert lo <= value <= hi


def test_just_inside_safe_boundary_is_safe():
    """A position just inside the margin-shrunk boundary is safe."""
    status, clamped = check_boundary(8.99, -8.99, 1.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'safe'
    assert clamped == pytest.approx((8.99, -8.99, 1.0))


def test_exact_safe_boundary_is_warning():
    """A position exactly at the margin-shrunk boundary is a warning, not safe."""
    status, _ = check_boundary(9.0, -9.0, 1.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'warning'


def test_zero_margin_only_reports_breach_or_safe():
    """With zero margin, any in-bounds position is safe, never a warning."""
    status, _ = check_boundary(9.9999, 9.9999, 19.9999, BOUNDS_MIN, BOUNDS_MAX, 0.0)
    assert status == 'safe'


def test_z_axis_breach_detected_independently():
    """A breach on the z axis alone is detected even if x and y are safe."""
    status, _ = check_boundary(0.0, 0.0, 25.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'
