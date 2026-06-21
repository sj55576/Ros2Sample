"""Tests for pure kinematics helpers in manipulator_sim."""

import math

import pytest

from manipulator_sim.kinematics import (
    clamp,
    forward_kinematics,
    inverse_kinematics,
    parse_targets_xy,
    step_towards,
    wrap_angle,
)


def test_parse_targets_xy_valid_pairs() -> None:
    """A flat x,y list should parse into tuple pairs."""
    parsed = parse_targets_xy([1.0, 2.0, 3.0, 4.0])
    assert parsed == [(1.0, 2.0), (3.0, 4.0)]


def test_parse_targets_xy_invalid_length() -> None:
    """An odd-length list should fail validation."""
    with pytest.raises(ValueError, match='targets_xy must contain one or more'):
        parse_targets_xy([1.0, 2.0, 3.0])


def test_inverse_and_forward_kinematics_round_trip() -> None:
    """Forward kinematics of IK output should recover the target point."""
    target_x, target_y = (1.0, 0.2)
    theta1, theta2 = inverse_kinematics(target_x, target_y, 0.8, 0.6)
    x, y = forward_kinematics(theta1, theta2, 0.8, 0.6)
    assert x == pytest.approx(target_x, abs=1e-6)
    assert y == pytest.approx(target_y, abs=1e-6)


def test_inverse_kinematics_unreachable_target() -> None:
    """Targets outside max reach should raise ValueError."""
    with pytest.raises(ValueError, match='unreachable'):
        inverse_kinematics(2.0, 0.0, 0.8, 0.6)


def test_step_towards_limits_delta() -> None:
    """step_towards should cap movement to max_delta."""
    assert step_towards(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert step_towards(0.0, 0.05, 0.1) == pytest.approx(0.05)
    assert step_towards(1.0, 0.0, 0.2) == pytest.approx(0.8)


def test_inverse_kinematics_elbow_branches_differ() -> None:
    """Elbow-up and elbow-down solutions should provide distinct second joint signs."""
    _, theta2_down = inverse_kinematics(1.0, 0.1, 0.8, 0.6, elbow_up=False)
    _, theta2_up = inverse_kinematics(1.0, 0.1, 0.8, 0.6, elbow_up=True)
    assert math.copysign(1.0, theta2_down) != math.copysign(1.0, theta2_up)


def test_clamp_limits_to_boundaries() -> None:
    """clamp should keep values inside the given range bounds."""
    assert clamp(0.5, 0.0, 1.0) == pytest.approx(0.5)
    assert clamp(-0.2, 0.0, 1.0) == pytest.approx(0.0)
    assert clamp(1.2, 0.0, 1.0) == pytest.approx(1.0)


def test_wrap_angle_normalizes_into_pi_range() -> None:
    """wrap_angle should normalize angles to [-pi, pi]."""
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(0.0) == pytest.approx(0.0)


def test_parse_targets_xy_rejects_non_finite_values() -> None:
    """NaN or infinite target coordinates should fail validation."""
    with pytest.raises(ValueError, match='finite values'):
        parse_targets_xy([0.5, float('nan')])


def test_inverse_kinematics_rejects_invalid_inputs() -> None:
    """IK should fail fast for non-finite coordinates and invalid link lengths."""
    with pytest.raises(ValueError, match='finite'):
        inverse_kinematics(float('inf'), 0.0, 0.8, 0.6)
    with pytest.raises(ValueError, match='link lengths must be positive'):
        inverse_kinematics(0.1, 0.0, 0.0, 0.6)
