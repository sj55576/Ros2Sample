"""Tests for the drone_sim.formation_utils module."""

from drone_sim.formation_utils import compute_formation_target, smooth_position
import pytest


def test_compute_formation_target_zero_offset():
    """Zero offset returns the leader position unchanged."""
    result = compute_formation_target(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    assert result == pytest.approx((1.0, 2.0, 3.0))


def test_compute_formation_target_positive_offset():
    """A positive offset is added to the leader position component-wise."""
    result = compute_formation_target(0.0, 0.0, 0.0, 1.0, 2.0, 3.0)
    assert result == pytest.approx((1.0, 2.0, 3.0))


def test_compute_formation_target_negative_offset():
    """A negative offset is subtracted from the leader position."""
    result = compute_formation_target(5.0, 5.0, 5.0, -2.0, -1.0, -3.0)
    assert result == pytest.approx((3.0, 4.0, 2.0))


def test_smooth_position_gain_zero_keeps_current():
    """A gain of zero returns the current position unchanged."""
    current = (1.0, 2.0, 3.0)
    target = (10.0, 20.0, 30.0)
    result = smooth_position(current, target, 0.0)
    assert result == pytest.approx(current)


def test_smooth_position_gain_one_jumps_to_target():
    """A gain of one returns the target position exactly."""
    current = (1.0, 2.0, 3.0)
    target = (10.0, 20.0, 30.0)
    result = smooth_position(current, target, 1.0)
    assert result == pytest.approx(target)


def test_smooth_position_gain_half_averages():
    """A gain of 0.5 averages current and target component-wise."""
    current = (0.0, 0.0, 0.0)
    target = (2.0, 4.0, 6.0)
    result = smooth_position(current, target, 0.5)
    assert result == pytest.approx((1.0, 2.0, 3.0))


def test_smooth_position_current_equals_target():
    """When current equals target, smoothing returns that position for any gain."""
    point = (3.0, 3.0, 3.0)
    result = smooth_position(point, point, 0.25)
    assert result == pytest.approx(point)
