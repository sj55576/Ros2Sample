"""Tests for the drone_sim.formation_controller module."""

import pytest

from drone_sim.formation_utils import compute_formation_target, smooth_position


def test_compute_formation_target_positive_offset():
    """Leader at (1,2,3) with offset (2,0,0) gives target (3,2,3)."""
    result = compute_formation_target(1.0, 2.0, 3.0, 2.0, 0.0, 0.0)
    assert result == pytest.approx((3.0, 2.0, 3.0))


def test_compute_formation_target_negative_offset():
    """Negative offset (-1,-1,0) is subtracted correctly from leader position."""
    result = compute_formation_target(5.0, 5.0, 5.0, -1.0, -1.0, 0.0)
    assert result == pytest.approx((4.0, 4.0, 5.0))


def test_compute_formation_target_zero_offset():
    """Zero offset returns the leader position unchanged."""
    result = compute_formation_target(3.0, 7.0, 2.0, 0.0, 0.0, 0.0)
    assert result == pytest.approx((3.0, 7.0, 2.0))


def test_smooth_position_gain_one():
    """Gain of 1.0 returns the target position exactly."""
    current = (0.0, 0.0, 0.0)
    target = (10.0, 20.0, 30.0)
    result = smooth_position(current, target, gain=1.0)
    assert result == pytest.approx((10.0, 20.0, 30.0))


def test_smooth_position_gain_zero():
    """Gain of 0.0 returns the current position exactly."""
    current = (1.0, 2.0, 3.0)
    target = (10.0, 20.0, 30.0)
    result = smooth_position(current, target, gain=0.0)
    assert result == pytest.approx((1.0, 2.0, 3.0))


def test_smooth_position_half_gain():
    """Gain of 0.5 returns the midpoint between current and target."""
    current = (0.0, 0.0, 0.0)
    target = (4.0, 8.0, 12.0)
    result = smooth_position(current, target, gain=0.5)
    assert result == pytest.approx((2.0, 4.0, 6.0))


def test_smooth_position_convergence():
    """Repeatedly applying smoothing converges the position toward the target."""
    current = (0.0, 0.0, 0.0)
    target = (100.0, 100.0, 100.0)
    pos = current
    for _ in range(50):
        pos = smooth_position(pos, target, gain=0.2)
    assert pos[0] == pytest.approx(target[0], rel=1e-2)
    assert pos[1] == pytest.approx(target[1], rel=1e-2)
    assert pos[2] == pytest.approx(target[2], rel=1e-2)
