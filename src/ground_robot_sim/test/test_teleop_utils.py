"""Tests for the ground_robot_sim.teleop_utils module."""

import pytest

from ground_robot_sim.teleop_utils import (
    MOVE_BINDINGS,
    SCALE_MAX,
    SCALE_MIN,
    clamp_scale,
    command_for_key,
    is_stop_key,
    scale_factor_for_key,
    scaled_velocity,
)


@pytest.mark.parametrize('key,expected', MOVE_BINDINGS.items())
def test_command_for_key_lowercase(key, expected):
    """Every lowercase MOVE_BINDINGS key maps to its bound command."""
    assert command_for_key(key) == expected


@pytest.mark.parametrize('key,expected', MOVE_BINDINGS.items())
def test_command_for_key_uppercase(key, expected):
    """Uppercase variants of MOVE_BINDINGS keys resolve case-insensitively."""
    assert command_for_key(key.upper()) == expected


def test_command_for_key_unbound():
    """An unbound key returns None."""
    assert command_for_key('k') is None


def test_scale_factor_for_key_q_lowercase():
    """'q' increases the scale factor."""
    assert scale_factor_for_key('q') == pytest.approx(1.1)


def test_scale_factor_for_key_q_uppercase():
    """'Q' resolves the same as 'q'."""
    assert scale_factor_for_key('Q') == pytest.approx(1.1)


def test_scale_factor_for_key_z_lowercase():
    """'z' decreases the scale factor."""
    assert scale_factor_for_key('z') == pytest.approx(0.9)


def test_scale_factor_for_key_z_uppercase():
    """'Z' resolves the same as 'z'."""
    assert scale_factor_for_key('Z') == pytest.approx(0.9)


def test_scale_factor_for_key_unbound():
    """An unbound key returns None."""
    assert scale_factor_for_key('w') is None


def test_is_stop_key_space():
    """Space is a stop key."""
    assert is_stop_key(' ') is True


def test_is_stop_key_x_lowercase():
    """'x' is a stop key."""
    assert is_stop_key('x') is True


def test_is_stop_key_x_uppercase():
    """'X' resolves the same as 'x'."""
    assert is_stop_key('X') is True


def test_is_stop_key_other():
    """A non-stop key returns False."""
    assert is_stop_key('w') is False


def test_clamp_scale_below_min():
    """Values below SCALE_MIN are clamped up to SCALE_MIN."""
    assert clamp_scale(0.0) == pytest.approx(SCALE_MIN)


def test_clamp_scale_above_max():
    """Values above SCALE_MAX are clamped down to SCALE_MAX."""
    assert clamp_scale(100.0) == pytest.approx(SCALE_MAX)


def test_clamp_scale_inside_bounds():
    """A value already within bounds is returned unchanged."""
    assert clamp_scale(1.0) == pytest.approx(1.0)


def test_scaled_velocity_basic():
    """Each command axis is multiplied by its corresponding scale."""
    result = scaled_velocity((1.0, 0.0, 0.0), 0.5, 0.5, 1.0)
    assert result == pytest.approx((0.5, 0.0, 0.0))


def test_scaled_velocity_all_axes():
    """All three axes scale independently."""
    result = scaled_velocity((1.0, 1.0, 1.0), 0.5, 2.0, 1.5)
    assert result == pytest.approx((0.5, 2.0, 1.5))


def test_scaled_velocity_negative():
    """Negative command components remain negative after scaling."""
    result = scaled_velocity((-1.0, -1.0, 1.0), 0.5, 0.5, 1.0)
    assert result == pytest.approx((-0.5, -0.5, 1.0))
