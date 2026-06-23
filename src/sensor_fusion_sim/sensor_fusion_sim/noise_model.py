"""Noise generation utilities for simulated sensors."""

import random
from typing import Tuple


def add_gaussian_noise(value: float, stddev: float) -> float:
    """Return value with additive Gaussian noise."""
    return value + random.gauss(0.0, stddev)


def add_gaussian_noise_3d(
    x: float,
    y: float,
    z: float,
    stddev_xy: float,
    stddev_z: float,
) -> Tuple[float, float, float]:
    """Return (x, y, z) with independent Gaussian noise per axis."""
    return (
        x + random.gauss(0.0, stddev_xy),
        y + random.gauss(0.0, stddev_xy),
        z + random.gauss(0.0, stddev_z),
    )


def drift_walk(current: float, stddev: float, limit: float) -> float:
    """Random-walk drift step, clamped to [-limit, limit]."""
    updated = current + random.gauss(0.0, stddev)
    return max(-limit, min(limit, updated))


def generate_imu_noise(
    ax: float,
    ay: float,
    az: float,
    gyro_z: float,
    accel_stddev: float,
    gyro_stddev: float,
    accel_bias: float = 0.0,
    gyro_bias: float = 0.0,
) -> Tuple[float, float, float, float]:
    """Add noise and bias to IMU readings, return (ax, ay, az, gyro_z)."""
    return (
        ax + accel_bias + random.gauss(0.0, accel_stddev),
        ay + accel_bias + random.gauss(0.0, accel_stddev),
        az + accel_bias + random.gauss(0.0, accel_stddev),
        gyro_z + gyro_bias + random.gauss(0.0, gyro_stddev),
    )
