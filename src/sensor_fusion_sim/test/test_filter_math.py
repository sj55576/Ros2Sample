"""Unit tests for sensor_fusion_sim.filter_math."""

import math

import pytest

from sensor_fusion_sim.filter_math import (
    complementary_filter_1d,
    complementary_filter_3d,
    dead_reckoning_step,
    euclidean_distance,
    innovation,
    normalize_angle,
)


def test_complementary_filter_1d_alpha_zero():
    """alpha=0 returns predicted value unchanged."""
    assert complementary_filter_1d(5.0, 10.0, 0.0) == pytest.approx(5.0)


def test_complementary_filter_1d_alpha_one():
    """alpha=1 returns measured value."""
    assert complementary_filter_1d(5.0, 10.0, 1.0) == pytest.approx(10.0)


def test_complementary_filter_1d_half():
    """alpha=0.5 returns average of predicted and measured."""
    assert complementary_filter_1d(4.0, 6.0, 0.5) == pytest.approx(5.0)


def test_complementary_filter_3d_alpha_one():
    """alpha=1 returns measured values in all three axes."""
    result = complementary_filter_3d(
        0.0, 0.0, 0.0,
        1.0, 2.0, 3.0,
        1.0,
    )
    assert result == pytest.approx((1.0, 2.0, 3.0))


def test_dead_reckoning_step_stationary():
    """Zero velocity returns the same (x, y) position."""
    x, y, yaw = dead_reckoning_step(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(2.0)


def test_dead_reckoning_step_forward():
    """vx=1.0, yaw=0, dt=1.0 increments x by 1."""
    x, y, yaw = dead_reckoning_step(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)


def test_normalize_angle_within_range():
    """Angle already in [-pi, pi] is returned unchanged."""
    assert normalize_angle(1.0) == pytest.approx(1.0)
    assert normalize_angle(-1.0) == pytest.approx(-1.0)


def test_normalize_angle_wraps_positive():
    """3*pi wraps to pi (magnitude)."""
    result = normalize_angle(3.0 * math.pi)
    assert abs(result) == pytest.approx(math.pi)


def test_normalize_angle_wraps_negative():
    """-3*pi wraps to -pi (magnitude)."""
    result = normalize_angle(-3.0 * math.pi)
    assert abs(result) == pytest.approx(math.pi)


def test_innovation_positive():
    """measured > predicted yields positive innovation."""
    assert innovation(5.0, 3.0) == pytest.approx(2.0)


def test_euclidean_distance_known():
    """Distance from (0,0) to (3,4) equals 5."""
    assert euclidean_distance(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)
