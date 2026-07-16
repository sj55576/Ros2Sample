"""Unit tests for the pure noise-injection helpers in noise_utils."""

import math
import random
import statistics

from ground_robot_sim.noise_utils import add_gaussian_noise, noisy_pose_2d, noisy_scan


def test_add_gaussian_noise_identity_at_zero_stddev():
    """Zero (or negative) stddev must return the input unchanged."""
    assert add_gaussian_noise(1.5, 0.0) == 1.5
    assert add_gaussian_noise(-2.0, -1.0) == -2.0


def test_add_gaussian_noise_deterministic_with_seeded_rng():
    """The same seeded RNG must produce the same noisy value."""
    rng_a = random.Random(42)
    rng_b = random.Random(42)
    assert add_gaussian_noise(1.0, 0.5, rng_a) == add_gaussian_noise(1.0, 0.5, rng_b)


def test_noisy_pose_2d_identity_at_zero_stddev():
    """Zero position and yaw stddev must leave the pose unchanged."""
    assert noisy_pose_2d(1.0, 2.0, 0.3, 0.0, 0.0) == (1.0, 2.0, 0.3)


def test_noisy_pose_2d_deterministic_with_seeded_rng():
    """The same seeded RNG must produce the same noisy pose."""
    rng_a = random.Random(7)
    rng_b = random.Random(7)
    pose_a = noisy_pose_2d(1.0, 2.0, 0.3, 0.1, 0.05, rng_a)
    pose_b = noisy_pose_2d(1.0, 2.0, 0.3, 0.1, 0.05, rng_b)
    assert pose_a == pose_b


def test_noisy_scan_identity_at_zero_stddev():
    """Zero stddev must leave scan ranges unchanged."""
    ranges = [1.0, 2.0, math.inf, float('nan')]
    result = noisy_scan(ranges, 0.0, 0.1, 5.0)
    assert result[:2] == ranges[:2]
    assert result[2] == math.inf
    assert math.isnan(result[3])


def test_noisy_scan_clamps_to_range():
    """Noisy ranges must always stay within [range_min, range_max]."""
    rng = random.Random(1)
    ranges = [0.1] * 200 + [5.0] * 200
    result = noisy_scan(ranges, 2.0, 0.2, 4.0, rng)
    assert all(0.2 <= value <= 4.0 for value in result)


def test_noisy_scan_passes_through_non_finite_values():
    """Inf and NaN readings must pass through unchanged even with noise enabled."""
    rng = random.Random(2)
    result = noisy_scan([math.inf, float('nan'), -math.inf], 1.0, 0.1, 8.0, rng)
    assert result[0] == math.inf
    assert math.isnan(result[1])
    assert result[2] == -math.inf


def test_noisy_scan_statistical_stddev_sanity():
    """A large sample of noisy readings must have approximately the requested stddev."""
    rng = random.Random(123)
    ranges = [4.0] * 2000
    result = noisy_scan(ranges, 1.0, 0.0, 100.0, rng)
    sample_stddev = statistics.stdev(result)
    assert 0.9 <= sample_stddev <= 1.1
