"""Mathematical utilities for complementary filtering."""

import math
from typing import Tuple


def complementary_filter_1d(
    predicted: float,
    measured: float,
    alpha: float,
) -> float:
    """Blend predicted and measured: alpha*measured + (1-alpha)*predicted."""
    return alpha * measured + (1.0 - alpha) * predicted


def complementary_filter_3d(
    pred_x: float,
    pred_y: float,
    pred_z: float,
    meas_x: float,
    meas_y: float,
    meas_z: float,
    alpha: float,
) -> Tuple[float, float, float]:
    """Blend 3D predicted and measured positions."""
    return (
        complementary_filter_1d(pred_x, meas_x, alpha),
        complementary_filter_1d(pred_y, meas_y, alpha),
        complementary_filter_1d(pred_z, meas_z, alpha),
    )


def dead_reckoning_step(
    x: float,
    y: float,
    yaw: float,
    vx: float,
    vy: float,
    yaw_rate: float,
    dt: float,
) -> Tuple[float, float, float]:
    """Predict next pose from current pose and velocity, return (x, y, yaw)."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_new = x + (vx * cos_yaw - vy * sin_yaw) * dt
    y_new = y + (vx * sin_yaw + vy * cos_yaw) * dt
    yaw_new = normalize_angle(yaw + yaw_rate * dt)
    return (x_new, y_new, yaw_new)


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def innovation(measured: float, predicted: float) -> float:
    """Compute measurement innovation (residual)."""
    return measured - predicted


def euclidean_distance(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    """Compute 2D Euclidean distance."""
    return math.hypot(x2 - x1, y2 - y1)
