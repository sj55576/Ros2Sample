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
    """Pi + 0.1 wraps to approximately -pi + 0.1."""
    result = normalize_angle(math.pi + 0.1)
    assert result == pytest.approx(-math.pi + 0.1, rel=1e-6)


def test_normalize_angle_wraps_below_negative_pi():
    """-pi - 0.1 wraps to approximately pi - 0.1."""
    result = normalize_angle(-math.pi - 0.1)
    assert result == pytest.approx(math.pi - 0.1, rel=1e-6)


def test_predict_zero_velocity_state_unchanged():
    """Zero velocity and yaw rate leave x, y, yaw unchanged."""
    x = np.array([1.0, 2.0, 0.5, 0.0, 0.0])
    p = np.eye(5) * 0.1
    q = make_process_noise(0.1, 0.1, 0.1, 0.1)
    x_pred, _ = predict(x, p, q, 1.0)
    np.testing.assert_allclose(x_pred, x, rtol=1e-6)


def test_predict_zero_velocity_covariance_grows_by_q():
    """Starting from zero covariance, p_pred equals q exactly."""
    x = np.array([1.0, 2.0, 0.5, 0.0, 0.0])
    p = np.zeros((5, 5))
    q = make_process_noise(0.1, 0.1, 0.1, 0.1)
    _, p_pred = predict(x, p, q, 1.0)
    np.testing.assert_allclose(p_pred, q, rtol=1e-6)


def test_predict_forward_motion():
    """Forward motion advances x, y by v*cos(yaw)*dt, v*sin(yaw)*dt."""
    x = np.array([0.0, 0.0, math.pi / 4.0, 2.0, 0.0])
    p = np.eye(5) * 0.1
    q = make_process_noise(0.01, 0.01, 0.01, 0.01)
    dt = 0.5
    x_pred, _ = predict(x, p, q, dt)
    expected_x = 2.0 * math.cos(math.pi / 4.0) * dt
    expected_y = 2.0 * math.sin(math.pi / 4.0) * dt
    assert x_pred[0] == pytest.approx(expected_x, rel=1e-6)
    assert x_pred[1] == pytest.approx(expected_y, rel=1e-6)


def test_predict_jacobian_matches_finite_difference():
    """Analytic p_pred matches a finite-difference Jacobian."""
    x = np.array([1.0, -2.0, 0.3, 1.5, 0.2])
    p = np.array([
        [0.5, 0.01, 0.0, 0.02, 0.0],
        [0.01, 0.4, 0.0, 0.0, 0.01],
        [0.0, 0.0, 0.2, 0.0, 0.0],
        [0.02, 0.0, 0.0, 0.3, 0.0],
        [0.0, 0.01, 0.0, 0.0, 0.1],
    ])
    q = np.zeros((5, 5))
    dt = 0.2

    x_pred, p_pred = predict(x, p, q, dt)

    eps = 1e-6
    f_numeric = np.zeros((5, 5))
    for j in range(5):
        x_perturbed = x.copy()
        x_perturbed[j] += eps
        x_pred_perturbed, _ = predict(x_perturbed, p, q, dt)
        diff = x_pred_perturbed - x_pred
        diff[2] = normalize_angle(diff[2])
        f_numeric[:, j] = diff / eps

    p_pred_numeric = f_numeric @ p @ f_numeric.T
    np.testing.assert_allclose(p_pred, p_pred_numeric, atol=1e-5)


def test_update_reduces_uncertainty():
    """A measurement update reduces total uncertainty (trace of p)."""
    x = initial_state()
    p = initial_covariance(1.0, 0.5, 0.5, 0.1)
    z, h = gps_measurement(0.5, 0.5)
    r = make_gps_noise(0.1)
    _, p_new = update(x, p, z, h, r)
    assert np.trace(p_new) < np.trace(p)


def test_update_converges_toward_measurement():
    """A precise measurement pulls the state estimate toward it."""
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    p = initial_covariance(1.0, 0.5, 0.5, 0.1)
    z, h = gps_measurement(5.0, -3.0)
    r = make_gps_noise(0.01)
    x_new, _ = update(x, p, z, h, r)
    assert x_new[0] == pytest.approx(5.0, abs=0.5)
    assert x_new[1] == pytest.approx(-3.0, abs=0.5)


def test_update_angle_wrapping_no_jump():
    """Innovation near +-pi wraps rather than jumping by 2*pi."""
    x = np.array([0.0, 0.0, math.pi - 0.05, 0.0, 0.0])
    p = initial_covariance(1.0, 0.5, 0.5, 0.1)
    z, h = imu_yaw_measurement(-math.pi + 0.05)
    r = make_imu_yaw_noise(0.01)
    x_new, _ = update(x, p, z, h, r, angle_indices=[0])
    assert abs(normalize_angle(x_new[2] - (math.pi - 0.05))) < 0.2


def test_gps_measurement_shapes_and_values():
    """gps_measurement returns z(2,), h(2,5) selecting x, y."""
    z, h = gps_measurement(1.5, -2.5)
    assert z.shape == (2,)
    assert h.shape == (2, 5)
    np.testing.assert_allclose(z, [1.5, -2.5])
    expected_h = np.zeros((2, 5))
    expected_h[0, 0] = 1.0
    expected_h[1, 1] = 1.0
    np.testing.assert_allclose(h, expected_h)


def test_odom_measurement_shapes_and_values():
    """odom_measurement returns z(3,), h(3,5) selecting x, y, v."""
    z, h = odom_measurement(1.0, 2.0, 3.0)
    assert z.shape == (3,)
    assert h.shape == (3, 5)
    np.testing.assert_allclose(z, [1.0, 2.0, 3.0])
    expected_h = np.zeros((3, 5))
    expected_h[0, 0] = 1.0
    expected_h[1, 1] = 1.0
    expected_h[2, 3] = 1.0
    np.testing.assert_allclose(h, expected_h)


def test_imu_gyro_measurement_shapes_and_values():
    """imu_gyro_measurement returns z(1,), h(1,5) selecting yaw_rate."""
    z, h = imu_gyro_measurement(0.75)
    assert z.shape == (1,)
    assert h.shape == (1, 5)
    np.testing.assert_allclose(z, [0.75])
    expected_h = np.zeros((1, 5))
    expected_h[0, 4] = 1.0
    np.testing.assert_allclose(h, expected_h)


def test_imu_yaw_measurement_shapes_and_values():
    """imu_yaw_measurement returns z(1,), h(1,5) selecting yaw."""
    z, h = imu_yaw_measurement(1.2)
    assert z.shape == (1,)
    assert h.shape == (1, 5)
    np.testing.assert_allclose(z, [1.2])
    expected_h = np.zeros((1, 5))
    expected_h[0, 2] = 1.0
    np.testing.assert_allclose(h, expected_h)


def test_make_process_noise_diagonal():
    """make_process_noise returns a diagonal matrix of squared stddevs."""
    q = make_process_noise(0.1, 0.2, 0.3, 0.4)
    assert q.shape == (5, 5)
    expected = np.diag([0.01, 0.01, 0.04, 0.09, 0.16])
    np.testing.assert_allclose(q, expected, rtol=1e-6)


def test_make_gps_noise_diagonal():
    """make_gps_noise returns a 2x2 diagonal of squared pos_std."""
    r = make_gps_noise(0.5)
    assert r.shape == (2, 2)
    np.testing.assert_allclose(r, np.diag([0.25, 0.25]), rtol=1e-6)


def test_make_odom_noise_diagonal():
    """make_odom_noise returns a 3x3 diagonal of squared stddevs."""
    r = make_odom_noise(0.2, 0.3)
    assert r.shape == (3, 3)
    np.testing.assert_allclose(r, np.diag([0.04, 0.04, 0.09]), rtol=1e-6)


def test_make_imu_gyro_noise():
    """make_imu_gyro_noise returns a 1x1 matrix of squared gyro_std."""
    r = make_imu_gyro_noise(0.1)
    assert r.shape == (1, 1)
    np.testing.assert_allclose(r, [[0.01]], rtol=1e-6)


def test_make_imu_yaw_noise():
    """make_imu_yaw_noise returns a 1x1 matrix of squared yaw_std."""
    r = make_imu_yaw_noise(0.2)
    assert r.shape == (1, 1)
    np.testing.assert_allclose(r, [[0.04]], rtol=1e-6)


def test_initial_state_zeros():
    """initial_state returns a zero vector of length 5."""
    x0 = initial_state()
    assert x0.shape == (5,)
    np.testing.assert_allclose(x0, np.zeros(5))


def test_initial_covariance_shape_and_values():
    """initial_covariance returns a diagonal matrix of squared stddevs."""
    p0 = initial_covariance(2.0, 1.0, 0.5, 0.25)
    assert p0.shape == (5, 5)
    expected = np.diag([4.0, 4.0, 1.0, 0.25, 0.0625])
    np.testing.assert_allclose(p0, expected, rtol=1e-6)


def test_full_predict_update_cycle_moves_toward_gps():
    """A predict then GPS update moves state toward z and shrinks p."""
    x = initial_state()
    p = initial_covariance(1.0, 0.5, 0.5, 0.1)
    q = make_process_noise(0.05, 0.05, 0.05, 0.05)

    x_pred, p_pred = predict(x, p, q, 0.1)

    z, h = gps_measurement(2.0, 1.0)
    r = make_gps_noise(0.05)
    x_new, p_new = update(x_pred, p_pred, z, h, r)

    assert abs(x_new[0] - 2.0) < abs(x_pred[0] - 2.0)
    assert abs(x_new[1] - 1.0) < abs(x_pred[1] - 1.0)
    assert np.trace(p_new) < np.trace(p_pred)
