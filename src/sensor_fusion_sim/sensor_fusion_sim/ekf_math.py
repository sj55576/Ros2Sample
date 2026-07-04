"""Extended Kalman Filter math for 2D planar motion.

State vector: [x, y, yaw, v, yaw_rate] (5 elements)
  x, y       — position in world frame
  yaw        — heading angle (rad)
  v          — forward speed in body frame
  yaw_rate   — angular velocity (rad/s)
"""

import math
from typing import List, Optional, Tuple

import numpy as np


STATE_DIM = 5
IDX_X = 0
IDX_Y = 1
IDX_YAW = 2
IDX_V = 3
IDX_YAW_RATE = 4


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def predict(
    x: np.ndarray,
    P: np.ndarray,
    Q: np.ndarray,
    dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """EKF predict step.

    Propagates state forward using a constant-velocity unicycle model
    and linearises via the Jacobian to propagate covariance.

    Returns (x_pred, P_pred).
    """
    yaw = x[IDX_YAW]
    v = x[IDX_V]
    yr = x[IDX_YAW_RATE]

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    x_pred = np.array([
        x[IDX_X] + v * cos_yaw * dt,
        x[IDX_Y] + v * sin_yaw * dt,
        normalize_angle(yaw + yr * dt),
        v,
        yr,
    ])

    F = np.eye(STATE_DIM)
    F[IDX_X, IDX_YAW] = -v * sin_yaw * dt
    F[IDX_X, IDX_V] = cos_yaw * dt
    F[IDX_Y, IDX_YAW] = v * cos_yaw * dt
    F[IDX_Y, IDX_V] = sin_yaw * dt
    F[IDX_YAW, IDX_YAW_RATE] = dt

    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred


def update(
    x: np.ndarray,
    P: np.ndarray,
    z: np.ndarray,
    H: np.ndarray,
    R: np.ndarray,
    angle_indices: Optional[List[int]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """EKF update step with a linear observation model.

    angle_indices: positions in z that represent angles (innovation
    is wrapped to [-pi, pi] for these).

    Returns (x_updated, P_updated).
    """
    y = z - H @ x
    if angle_indices:
        for i in angle_indices:
            y[i] = normalize_angle(y[i])

    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    x_new = x + K @ y
    x_new[IDX_YAW] = normalize_angle(x_new[IDX_YAW])

    P_new = (np.eye(STATE_DIM) - K @ H) @ P
    return x_new, P_new


# ── Measurement helpers ────────────────────────────────────────────


def gps_measurement(
    x_gps: float, y_gps: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build z and H for a GPS position measurement [x, y]."""
    z = np.array([x_gps, y_gps])
    H = np.zeros((2, STATE_DIM))
    H[0, IDX_X] = 1.0
    H[1, IDX_Y] = 1.0
    return z, H


def odom_measurement(
    x_odom: float, y_odom: float, v_odom: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build z and H for a wheel-odometry measurement [x, y, v]."""
    z = np.array([x_odom, y_odom, v_odom])
    H = np.zeros((3, STATE_DIM))
    H[0, IDX_X] = 1.0
    H[1, IDX_Y] = 1.0
    H[2, IDX_V] = 1.0
    return z, H


def imu_gyro_measurement(yaw_rate: float) -> Tuple[np.ndarray, np.ndarray]:
    """Build z and H for an IMU gyroscope measurement [yaw_rate]."""
    z = np.array([yaw_rate])
    H = np.zeros((1, STATE_DIM))
    H[0, IDX_YAW_RATE] = 1.0
    return z, H


def imu_yaw_measurement(yaw: float) -> Tuple[np.ndarray, np.ndarray]:
    """Build z and H for an IMU orientation measurement [yaw]."""
    z = np.array([yaw])
    H = np.zeros((1, STATE_DIM))
    H[0, IDX_YAW] = 1.0
    return z, H


# ── Noise covariance factories ─────────────────────────────────────


def make_process_noise(
    pos_std: float,
    yaw_std: float,
    vel_std: float,
    yaw_rate_std: float,
) -> np.ndarray:
    """Diagonal process-noise covariance Q (5×5)."""
    return np.diag([
        pos_std ** 2,
        pos_std ** 2,
        yaw_std ** 2,
        vel_std ** 2,
        yaw_rate_std ** 2,
    ])


def make_gps_noise(pos_std: float) -> np.ndarray:
    """GPS measurement-noise covariance R (2×2)."""
    return np.diag([pos_std ** 2, pos_std ** 2])


def make_odom_noise(pos_std: float, vel_std: float) -> np.ndarray:
    """Wheel-odometry measurement-noise covariance R (3×3)."""
    return np.diag([pos_std ** 2, pos_std ** 2, vel_std ** 2])


def make_imu_gyro_noise(gyro_std: float) -> np.ndarray:
    """IMU gyro measurement-noise covariance R (1×1)."""
    return np.array([[gyro_std ** 2]])


def make_imu_yaw_noise(yaw_std: float) -> np.ndarray:
    """IMU yaw measurement-noise covariance R (1×1)."""
    return np.array([[yaw_std ** 2]])


def initial_state() -> np.ndarray:
    """Zero initial state vector."""
    return np.zeros(STATE_DIM)


def initial_covariance(
    pos_std: float = 1.0,
    yaw_std: float = 0.5,
    vel_std: float = 0.5,
    yaw_rate_std: float = 0.1,
) -> np.ndarray:
    """Diagonal initial covariance P0."""
    return np.diag([
        pos_std ** 2,
        pos_std ** 2,
        yaw_std ** 2,
        vel_std ** 2,
        yaw_rate_std ** 2,
    ])
