"""Tests for the drone_sim.noise_utils module."""

import random
import statistics

from drone_sim.noise_utils import add_gaussian_noise, noisy_imu, noisy_xyz
import pytest


def test_add_gaussian_noise_zero_stddev_is_identity():
    """Zero stddev returns the value unchanged."""
    assert add_gaussian_noise(3.5, 0.0) == pytest.approx(3.5)


def test_add_gaussian_noise_negative_stddev_is_identity():
    """Negative stddev also returns the value unchanged."""
    assert add_gaussian_noise(-2.0, -1.0) == pytest.approx(-2.0)


def test_add_gaussian_noise_seeded_deterministic():
    """A seeded rng produces reproducible noise."""
    rng_a = random.Random(42)
    rng_b = random.Random(42)
    assert add_gaussian_noise(1.0, 1.0, rng=rng_a) == add_gaussian_noise(1.0, 1.0, rng=rng_b)


def test_add_gaussian_noise_statistical_stddev():
    """Sample stddev over many draws is close to the requested stddev."""
    rng = random.Random(7)
    samples = [add_gaussian_noise(0.0, 1.0, rng=rng) for _ in range(2000)]
    sample_stddev = statistics.stdev(samples)
    assert 0.9 <= sample_stddev <= 1.1


def test_noisy_xyz_zero_stddev_is_identity():
    """Zero stddev leaves all three axes unchanged."""
    assert noisy_xyz(1.0, 2.0, 3.0, 0.0) == (1.0, 2.0, 3.0)


def test_noisy_xyz_seeded_deterministic():
    """A seeded rng produces reproducible xyz noise."""
    rng_a = random.Random(11)
    rng_b = random.Random(11)
    assert noisy_xyz(1.0, 2.0, 3.0, 0.5, rng=rng_a) == noisy_xyz(1.0, 2.0, 3.0, 0.5, rng=rng_b)


def test_noisy_imu_zero_stddev_and_bias_is_identity():
    """Zero noise stddev and zero bias leaves IMU readings unchanged."""
    result = noisy_imu(0.1, 0.2, 9.8, 0.05, 0.0, 0.0, gyro_bias=0.0)
    assert result == (0.1, 0.2, 9.8, 0.05)


def test_noisy_imu_gyro_bias_shifts_mean():
    """A gyro bias shifts the sampled mean of the gyro reading."""
    rng = random.Random(3)
    samples = [
        noisy_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.2, gyro_bias=1.0, rng=rng)[3]
        for _ in range(2000)
    ]
    mean = statistics.mean(samples)
    assert mean == pytest.approx(1.0, abs=0.1)


def test_noisy_imu_seeded_deterministic():
    """A seeded rng produces reproducible IMU noise."""
    rng_a = random.Random(21)
    rng_b = random.Random(21)
    result_a = noisy_imu(0.1, 0.2, 9.8, 0.05, 0.3, 0.1, gyro_bias=0.02, rng=rng_a)
    result_b = noisy_imu(0.1, 0.2, 9.8, 0.05, 0.3, 0.1, gyro_bias=0.02, rng=rng_b)
    assert result_a == result_b
