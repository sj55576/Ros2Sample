"""Tests for analytical inverse kinematics in manipulator_sim."""

import math

import pytest

from manipulator_sim.inverse_kinematics import (
    nearest_reachable,
    solve_ik,
    workspace_radius,
)
from manipulator_sim.kinematics import forward_kinematics


def test_forward_inverse_roundtrip() -> None:
    """FK then IK then FK should recover the original end-effector position."""
    l1, l2 = 0.8, 0.6
    target_x, target_y = 1.0, 0.2
    result = solve_ik(target_x, target_y, l1, l2)
    assert result is not None
    q1, q2 = result
    x, y = forward_kinematics(q1, q2, l1, l2)
    assert x == pytest.approx(target_x, abs=1e-6)
    assert y == pytest.approx(target_y, abs=1e-6)


def test_unreachable_too_far() -> None:
    """Target beyond l1+l2 should return None."""
    l1, l2 = 0.8, 0.6
    result = solve_ik(2.0, 0.0, l1, l2)
    assert result is None


def test_unreachable_too_close() -> None:
    """Target inside abs(l1-l2) should return None."""
    l1, l2 = 0.8, 0.3
    # abs(l1 - l2) = 0.5; target at distance 0.1 is too close
    result = solve_ik(0.1, 0.0, l1, l2)
    assert result is None


def test_elbow_up_vs_down() -> None:
    """Elbow-up and elbow-down should give valid but distinct solutions."""
    l1, l2 = 0.8, 0.6
    x, y = 1.0, 0.1
    up = solve_ik(x, y, l1, l2, elbow_up=True)
    down = solve_ik(x, y, l1, l2, elbow_up=False)
    assert up is not None
    assert down is not None
    # The second joint angle signs should differ
    assert math.copysign(1.0, up[1]) != math.copysign(1.0, down[1])
    # Both should reach the target via FK
    xu, yu = forward_kinematics(up[0], up[1], l1, l2)
    xd, yd = forward_kinematics(down[0], down[1], l1, l2)
    assert xu == pytest.approx(x, abs=1e-6)
    assert yu == pytest.approx(y, abs=1e-6)
    assert xd == pytest.approx(x, abs=1e-6)
    assert yd == pytest.approx(y, abs=1e-6)


def test_fully_extended() -> None:
    """Target at exactly l1+l2 distance should give q2 approximately zero."""
    l1, l2 = 0.8, 0.6
    result = solve_ik(l1 + l2, 0.0, l1, l2)
    assert result is not None
    q1, q2 = result
    assert q2 == pytest.approx(0.0, abs=1e-6)


def test_workspace_radius() -> None:
    """workspace_radius should return the correct annulus boundaries."""
    l1, l2 = 0.8, 0.6
    r_min, r_max = workspace_radius(l1, l2)
    assert r_min == pytest.approx(abs(l1 - l2), abs=1e-9)
    assert r_max == pytest.approx(l1 + l2, abs=1e-9)


def test_nearest_reachable_inside_workspace() -> None:
    """A point already in the workspace should be returned unchanged."""
    l1, l2 = 0.8, 0.6
    x, y = 0.9, 0.3
    cx, cy = nearest_reachable(x, y, l1, l2)
    assert cx == pytest.approx(x, abs=1e-9)
    assert cy == pytest.approx(y, abs=1e-9)


def test_nearest_reachable_too_far() -> None:
    """A point beyond max reach should be projected onto the outer boundary."""
    l1, l2 = 0.8, 0.6
    r_max = l1 + l2
    cx, cy = nearest_reachable(2.0, 0.0, l1, l2)
    assert math.hypot(cx, cy) == pytest.approx(r_max, abs=1e-9)
    assert cx == pytest.approx(r_max, abs=1e-9)
    assert cy == pytest.approx(0.0, abs=1e-9)


def test_nearest_reachable_too_close() -> None:
    """A point inside min reach should be projected onto the inner boundary."""
    l1, l2 = 0.8, 0.3
    r_min = abs(l1 - l2)
    cx, cy = nearest_reachable(0.1, 0.0, l1, l2)
    assert math.hypot(cx, cy) == pytest.approx(r_min, abs=1e-9)


def test_nearest_reachable_at_origin() -> None:
    """A target at the exact origin should not divide by zero."""
    l1, l2 = 0.8, 0.6
    cx, cy = nearest_reachable(0.0, 0.0, l1, l2)
    # Should return the max-reach point along the positive x-axis
    assert cx == pytest.approx(l1 + l2, abs=1e-9)
    assert cy == pytest.approx(0.0, abs=1e-9)


def test_known_geometry() -> None:
    """l1=l2=1, target=(1,1): elbow-down gives q1=0, q2=pi/2; elbow-up gives q1=pi/2, q2=-pi/2."""
    l1, l2 = 1.0, 1.0
    x, y = 1.0, 1.0
    down = solve_ik(x, y, l1, l2, elbow_up=False)
    up = solve_ik(x, y, l1, l2, elbow_up=True)
    assert down is not None
    assert up is not None
    assert down[0] == pytest.approx(0.0, abs=1e-6)
    assert down[1] == pytest.approx(math.pi / 2.0, abs=1e-6)
    assert up[0] == pytest.approx(math.pi / 2.0, abs=1e-6)
    assert up[1] == pytest.approx(-math.pi / 2.0, abs=1e-6)
