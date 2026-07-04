"""Unit tests for sensor_fusion_sim.ekf_math."""

import math

import numpy as np
import pytest

from sensor_fusion_sim.ekf_math import (
    gps_measurement,
    imu_gyro_measurement,
    imu_yaw_measurement,
    initial_covariance,
    initial_state,
    make_gps_noise,
    make_imu_gyro_noise,
    make_imu_yaw_noise,
    make_odom_noise,
    make_process_noise,
    normalize_angle,
    odom_measurement,
    predict,
    update,
)


def test_normalize_angle_zero():
    """0 maps to 0."""
    assert normalize_angle(0.0) == pytest.approx(0.0, abs=1e-9)


def test_normalize_angle_two_pi():
    """2*pi wraps to 0."""
    assert normalize_angle(2.0 * math.pi) == pytest.approx(0.0, abs=1e-9)


def test_normalize_angle_negative_two_pi():
    """-2*pi wraps to 0."""
    assert normalize_angle(-2.0 * math.pi) == pytest.approx(0.0, abs=1e-9)


def test_normalize_angle_wraps_above_pi():
    """pi + 0.1 wraps to approximately -pi + 0.1."""
    result = normalize_angle(math.pi + 0.1)
    assert result == pytest.approx(-math.pi + 0.1, rel=1e-6)


def test_normalize_angle_wraps_below_negative_pi():
    """-pi - 0.1 wraps to approximately pi - 0.1."""
    result = normalize_angle(-math.pi - 0.1)
    assert result == pytest.approx(math.pi - 0.1, rel=1e-6)


def test_predict_zero_velocity_state_unchanged():
    """Zero velocity and yaw rate leave x, y, yaw unchanged."""
    x = np.array([1.0, 2.0, 0.5, 0.0, 0.0])
    P = np.eye(5) * 0.1
    Q = make_process_noise(0.1, 0.1, 0.1, 0.1)
    x_pred, _ = predict(x, P, Q, 1.0)
    np.testing.assert_allclose(x_pred, x, rtol=1e-6)


def test_predict_zero_velocity_covariance_grows_by_q():
    """Starting from zero covariance, P_pred equals Q exactly."""
    x = np.array([1.0, 2.0, 0.5, 0.0, 0.0])
    P = np.zeros((5, 5))
    Q = make_process_noise(0.1, 0.1, 0.1, 0.1)
    _, P_pred = predict(x, P, Q, 1.0)
    np.testing.assert_allclose(P_pred, Q, rtol=1e-6)


def test_predict_forward_motion():
    """Forward motion advances x, y by v*cos(yaw)*dt, v*sin(yaw)*dt."""
    x = np.array([0.0, 0.0, math.pi / 4.0, 2.0, 0.0])
    P = np.eye(5) * 0.1
    Q = make_process_noise(0.01, 0.01, 0.01, 0.01)
    dt = 0.5
    x_pred, _ = predict(x, P, Q, dt)
    expected_x = 2.0 * math.cos(math.pi / 4.0) * dt
    expected_y = 2.0 * math.sin(math.pi / 4.0) * dt
    assert x_pred[0] == pytest.approx(expected_x, rel=1e-6)
    assert x_pred[1] == pytest.approx(expected_y, rel=1e-6)


def test_predict_jacobian_matches_finite_difference():
    """Analytic P_pred matches a finite-difference Jacobian."""
    x = np.array([1.0, -2.0, 0.3, 1.5, 0.2])
    P = np.array([
        [0.5, 0.01, 0.0, 0.02, 0.0],
        [0.01, 0.4, 0.0, 0.0, 0.01],
        [0.0, 0.0, 0.2, 0.0, 0.0],
        [0.02, 0.0, 0.0, 0.3, 0.0],
        [0.0, 0.01, 0.0, 0.0, 0.1],
    ])
    Q = np.zeros((5, 5))
    dt = 0.2

    x_pred, P_pred = predict(x, P, Q, dt)

    eps = 1e-6
    F_numeric = np.zeros((5, 5))
    for j in range(5):
        x_perturbed = x.copy()
        x_perturbed[j] += eps
        x_pred_perturbed, _ = predict(x_perturbed, P, Q, dt)
        diff = x_pred_perturbed - x_pred
        diff[2] = normalize_angle(diff[2])
        F_numeric[:, j] = diff / eps

    P_pred_numeric = F_numeric @ P @ F_numeric.T
    np.testing.assert_allclose(P_pred, P_pred_numeric, atol=1e-5)


def test_update_reduces_uncertainty():
    """A measurement update reduces total uncertainty (trace of P)."""
    x = initial_state()
    P = initial_covariance(1.0, 0.5, 0.5, 0.1)
    z, H = gps_measurement(0.5, 0.5)
    R = make_gps_noise(0.1)
    _, P_new = update(x, P, z, H, R)
    assert np.trace(P_new) < np.trace(P)


def test_update_converges_toward_measurement():
    """A precise measurement pulls the state estimate toward it."""
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    P = initial_covariance(1.0, 0.5, 0.5, 0.1)
    z, H = gps_measurement(5.0, -3.0)
    R = make_gps_noise(0.01)
    x_new, _ = update(x, P, z, H, R)
    assert x_new[0] == pytest.approx(5.0, abs=0.5)
    assert x_new[1] == pytest.approx(-3.0, abs=0.5)


def test_update_angle_wrapping_no_jump():
    """Innovation near +-pi wraps rather than jumping by 2*pi."""
    x = np.array([0.0, 0.0, math.pi - 0.05, 0.0, 0.0])
    P = initial_covariance(1.0, 0.5, 0.5, 0.1)
    z, H = imu_yaw_measurement(-math.pi + 0.05)
    R = make_imu_yaw_noise(0.01)
    x_new, _ = update(x, P, z, H, R, angle_indices=[0])
    assert abs(normalize_angle(x_new[2] - (math.pi - 0.05))) < 0.2


def test_gps_measurement_shapes_and_values():
    """gps_measurement returns z(2,), H(2,5) selecting x, y."""
    z, H = gps_measurement(1.5, -2.5)
    assert z.shape == (2,)
    assert H.shape == (2, 5)
    np.testing.assert_allclose(z, [1.5, -2.5])
    expected_H = np.zeros((2, 5))
    expected_H[0, 0] = 1.0
    expected_H[1, 1] = 1.0
    np.testing.assert_allclose(H, expected_H)


def test_odom_measurement_shapes_and_values():
    """odom_measurement returns z(3,), H(3,5) selecting x, y, v."""
    z, H = odom_measurement(1.0, 2.0, 3.0)
    assert z.shape == (3,)
    assert H.shape == (3, 5)
    np.testing.assert_allclose(z, [1.0, 2.0, 3.0])
    expected_H = np.zeros((3, 5))
    expected_H[0, 0] = 1.0
    expected_H[1, 1] = 1.0
    expected_H[2, 3] = 1.0
    np.testing.assert_allclose(H, expected_H)


def test_imu_gyro_measurement_shapes_and_values():
    """imu_gyro_measurement returns z(1,), H(1,5) selecting yaw_rate."""
    z, H = imu_gyro_measurement(0.75)
    assert z.shape == (1,)
    assert H.shape == (1, 5)
    np.testing.assert_allclose(z, [0.75])
    expected_H = np.zeros((1, 5))
    expected_H[0, 4] = 1.0
    np.testing.assert_allclose(H, expected_H)


def test_imu_yaw_measurement_shapes_and_values():
    """imu_yaw_measurement returns z(1,), H(1,5) selecting yaw."""
    z, H = imu_yaw_measurement(1.2)
    assert z.shape == (1,)
    assert H.shape == (1, 5)
    np.testing.assert_allclose(z, [1.2])
    expected_H = np.zeros((1, 5))
    expected_H[0, 2] = 1.0
    np.testing.assert_allclose(H, expected_H)


def test_make_process_noise_diagonal():
    """make_process_noise returns a diagonal matrix of squared stddevs."""
    Q = make_process_noise(0.1, 0.2, 0.3, 0.4)
    assert Q.shape == (5, 5)
    expected = np.diag([0.01, 0.01, 0.04, 0.09, 0.16])
    np.testing.assert_allclose(Q, expected, rtol=1e-6)


def test_make_gps_noise_diagonal():
    """make_gps_noise returns a 2x2 diagonal of squared pos_std."""
    R = make_gps_noise(0.5)
    assert R.shape == (2, 2)
    np.testing.assert_allclose(R, np.diag([0.25, 0.25]), rtol=1e-6)


def test_make_odom_noise_diagonal():
    """make_odom_noise returns a 3x3 diagonal of squared stddevs."""
    R = make_odom_noise(0.2, 0.3)
    assert R.shape == (3, 3)
    np.testing.assert_allclose(R, np.diag([0.04, 0.04, 0.09]), rtol=1e-6)


def test_make_imu_gyro_noise():
    """make_imu_gyro_noise returns a 1x1 matrix of squared gyro_std."""
    R = make_imu_gyro_noise(0.1)
    assert R.shape == (1, 1)
    np.testing.assert_allclose(R, [[0.01]], rtol=1e-6)


def test_make_imu_yaw_noise():
    """make_imu_yaw_noise returns a 1x1 matrix of squared yaw_std."""
    R = make_imu_yaw_noise(0.2)
    assert R.shape == (1, 1)
    np.testing.assert_allclose(R, [[0.04]], rtol=1e-6)


def test_initial_state_zeros():
    """initial_state returns a zero vector of length 5."""
    x0 = initial_state()
    assert x0.shape == (5,)
    np.testing.assert_allclose(x0, np.zeros(5))


def test_initial_covariance_shape_and_values():
    """initial_covariance returns a diagonal matrix of squared stddevs."""
    P0 = initial_covariance(2.0, 1.0, 0.5, 0.25)
    assert P0.shape == (5, 5)
    expected = np.diag([4.0, 4.0, 1.0, 0.25, 0.0625])
    np.testing.assert_allclose(P0, expected, rtol=1e-6)


def test_full_predict_update_cycle_moves_toward_gps():
    """A predict then GPS update moves state toward z and shrinks P."""
    x = initial_state()
    P = initial_covariance(1.0, 0.5, 0.5, 0.1)
    Q = make_process_noise(0.05, 0.05, 0.05, 0.05)

    x_pred, P_pred = predict(x, P, Q, 0.1)

    z, H = gps_measurement(2.0, 1.0)
    R = make_gps_noise(0.05)
    x_new, P_new = update(x_pred, P_pred, z, H, R)

    assert abs(x_new[0] - 2.0) < abs(x_pred[0] - 2.0)
    assert abs(x_new[1] - 1.0) < abs(x_pred[1] - 1.0)
    assert np.trace(P_new) < np.trace(P_pred)
