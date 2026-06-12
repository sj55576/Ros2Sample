"""Small math helpers used by the drone simulation nodes."""

from math import atan2, cos, sin
from typing import Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to the inclusive [lower, upper] range."""
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return atan2(sin(angle), cos(angle))


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion from roll, pitch, and yaw radians."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = cos(half_roll)
    sr = sin(half_roll)
    cp = cos(half_pitch)
    sp = sin(half_pitch)
    cy = cos(half_yaw)
    sy = sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
