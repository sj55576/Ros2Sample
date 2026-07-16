"""Pure-function Gaussian noise helpers for simulated drone sensors."""

import random
from typing import Optional, Tuple


def add_gaussian_noise(
    value: float,
    stddev: float,
    rng: Optional[random.Random] = None,
) -> float:
    """
    Return value with additive Gaussian noise.

    No random call is made and the value is returned unchanged when
    stddev is less than or equal to 0.0.
    """
    if stddev <= 0.0:
        return value
    generator = rng if rng is not None else random
    return value + generator.gauss(0.0, stddev)


def noisy_xyz(
    x: float,
    y: float,
    z: float,
    stddev: float,
    rng: Optional[random.Random] = None,
) -> Tuple[float, float, float]:
    """
    Return (x, y, z) with independent Gaussian noise applied per axis.

    Identity when stddev is less than or equal to 0.0.
    """
    return (
        add_gaussian_noise(x, stddev, rng),
        add_gaussian_noise(y, stddev, rng),
        add_gaussian_noise(z, stddev, rng),
    )


def noisy_imu(
    ax: float,
    ay: float,
    az: float,
    gyro_z: float,
    accel_stddev: float,
    gyro_stddev: float,
    gyro_bias: float = 0.0,
    rng: Optional[random.Random] = None,
) -> Tuple[float, float, float, float]:
    """
    Add noise and gyro bias to IMU readings, return (ax, ay, az, gyro_z).

    Acceleration axes get independent additive Gaussian noise. The gyro
    reading is offset by a constant bias plus additive Gaussian noise.
    """
    noisy_ax = add_gaussian_noise(ax, accel_stddev, rng)
    noisy_ay = add_gaussian_noise(ay, accel_stddev, rng)
    noisy_az = add_gaussian_noise(az, accel_stddev, rng)
    noisy_gyro_z = add_gaussian_noise(gyro_z + gyro_bias, gyro_stddev, rng)
    return (noisy_ax, noisy_ay, noisy_az, noisy_gyro_z)
