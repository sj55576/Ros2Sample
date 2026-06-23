"""Unit tests for sensor_fusion_sim.noise_model."""

import pytest

from sensor_fusion_sim.noise_model import (
    add_gaussian_noise,
    add_gaussian_noise_3d,
    drift_walk,
    generate_imu_noise,
)


def test_add_gaussian_noise_mean_approx_zero():
    """Mean of many noise samples is close to zero."""
    n = 10000
    samples = [add_gaussian_noise(0.0, 1.0) for _ in range(n)]
    mean = sum(samples) / n
    assert abs(mean) < 0.1


def test_add_gaussian_noise_zero_stddev():
    """With stddev=0, output equals input."""
    assert add_gaussian_noise(3.14, 0.0) == pytest.approx(3.14)


def test_add_gaussian_noise_3d_zero_stddev():
    """With zero noise, 3D output equals input exactly."""
    result = add_gaussian_noise_3d(1.0, 2.0, 3.0, 0.0, 0.0)
    assert result == pytest.approx((1.0, 2.0, 3.0))


def test_add_gaussian_noise_3d_adds_noise():
    """With nonzero stddev, 3D output differs from input."""
    found_diff = False
    for _ in range(20):
        rx, ry, rz = add_gaussian_noise_3d(0.0, 0.0, 0.0, 1.0, 1.0)
        if rx != 0.0 or ry != 0.0 or rz != 0.0:
            found_diff = True
            break
    assert found_diff


def test_drift_walk_stays_within_limits():
    """drift_walk with limit=1.0 always stays in [-1, 1]."""
    value = 0.0
    for _ in range(1000):
        value = drift_walk(value, 0.1, 1.0)
        assert -1.0 <= value <= 1.0


def test_drift_walk_zero_stddev_no_change():
    """With stddev=0, drift_walk returns unchanged value."""
    assert drift_walk(0.5, 0.0, 1.0) == pytest.approx(0.5)


def test_generate_imu_noise_zero_noise():
    """With stddev=0 and bias=0, output equals input."""
    result = generate_imu_noise(1.0, 2.0, 3.0, 0.1, 0.0, 0.0)
    assert result == pytest.approx((1.0, 2.0, 3.0, 0.1))


def test_generate_imu_noise_adds_bias():
    """With accel_bias=1.0 and stddev=0, output = input + 1.0."""
    result = generate_imu_noise(
        0.0, 0.0, 0.0, 0.0,
        accel_stddev=0.0,
        gyro_stddev=0.0,
        accel_bias=1.0,
    )
    assert result == pytest.approx((1.0, 1.0, 1.0, 0.0))
