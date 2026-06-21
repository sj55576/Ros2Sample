"""Tests for the ground_robot_sim.geometry module."""

import math

import pytest

from ground_robot_sim.geometry import (
    normalize_angle,
    parse_circles,
    ray_circle_distance,
    yaw_to_quaternion,
)


def test_normalize_angle_zero():
    """Zero stays zero."""
    assert normalize_angle(0.0) == pytest.approx(0.0)


def test_normalize_angle_three_pi():
    """3*pi wraps to approximately +/-pi (magnitude pi)."""
    assert abs(normalize_angle(3.0 * math.pi)) == pytest.approx(math.pi)


def test_normalize_angle_minus_three_halves_pi():
    """Negative 3*pi/2 wraps to pi/2."""
    assert normalize_angle(-3.0 * math.pi / 2.0) == pytest.approx(math.pi / 2.0)


def test_normalize_angle_already_in_range():
    """Value already in [-pi, pi] is returned unchanged."""
    assert normalize_angle(1.0) == pytest.approx(1.0)


def test_yaw_to_quaternion_quarter_pi():
    """Yaw pi/4 gives the correct sin/cos half-angle quaternion."""
    yaw = math.pi / 4.0
    x, y, z, w = yaw_to_quaternion(yaw)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(math.sin(yaw / 2.0))
    assert w == pytest.approx(math.cos(yaw / 2.0))


def test_yaw_to_quaternion_zero():
    """Yaw 0 gives the identity quaternion for the z/w components."""
    x, y, z, w = yaw_to_quaternion(0.0)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(0.0)
    assert w == pytest.approx(1.0)


def test_parse_circles_valid():
    """A multiple-of-3 flat list parses into correct circle tuples."""
    result = parse_circles([1.0, 2.0, 0.5, 3.0, 4.0, 1.0])
    assert result == [(1.0, 2.0, 0.5), (3.0, 4.0, 1.0)]


def test_parse_circles_bad_length():
    """A list whose length is not a multiple of 3 raises ValueError."""
    with pytest.raises(ValueError, match='obstacles parameter length must be a multiple of 3'):
        parse_circles([1.0, 2.0])


def test_ray_circle_distance_hit():
    """Ray from origin along +x to circle at (2, 0, r=0.5) returns approx 1.5."""
    dist = ray_circle_distance(0.0, 0.0, 1.0, 0.0, (2.0, 0.0, 0.5))
    assert dist == pytest.approx(1.5)


def test_ray_circle_distance_pointing_away():
    """Ray pointing in the -x direction away from circle returns inf."""
    dist = ray_circle_distance(0.0, 0.0, -1.0, 0.0, (2.0, 0.0, 0.5))
    assert math.isinf(dist)


def test_ray_circle_distance_inside_circle():
    """Ray starting inside a circle returns the forward exit distance."""
    # Origin at (1.9, 0) inside circle at (2, 0, r=0.5); ray along +x.
    # The forward intersection is at x = 2.5, so distance ≈ 0.6.
    dist = ray_circle_distance(1.9, 0.0, 1.0, 0.0, (2.0, 0.0, 0.5))
    assert dist == pytest.approx(0.6, abs=1e-9)


def test_ray_circle_distance_miss():
    """Ray that misses the circle entirely returns inf."""
    # Ray along +y starting at origin; circle at (2, 0, 0.5) is off to the side.
    dist = ray_circle_distance(0.0, 0.0, 0.0, 1.0, (2.0, 0.0, 0.5))
    assert math.isinf(dist)


def test_parse_circles_rejects_non_finite_values():
    """Obstacle definitions should reject NaN or infinite values."""
    with pytest.raises(ValueError, match='finite values'):
        parse_circles([0.0, float('inf'), 0.5])


def test_parse_circles_rejects_non_positive_radius():
    """Obstacle radii must be positive to avoid invalid range simulation geometry."""
    with pytest.raises(ValueError, match='radii must be positive'):
        parse_circles([0.0, 0.0, 0.0])
