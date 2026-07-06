"""Tests for the ground_robot_sim.pid module."""

from ground_robot_sim.pid import PIDController
import pytest


def test_p_only():
    """P-only controller output equals kp * error."""
    pid = PIDController(kp=2.0)
    result = pid.compute(error=3.0, dt=0.1)
    assert result == pytest.approx(2.0 * 3.0)


def test_pi_integral_accumulates():
    """PI controller accumulates integral over multiple steps."""
    pid = PIDController(kp=1.0, ki=1.0)
    pid.compute(error=1.0, dt=0.1)
    result = pid.compute(error=1.0, dt=0.1)
    # After two steps: integral = 0.1 + 0.1 = 0.2; output = 1.0 + 0.2 = 1.2
    assert result == pytest.approx(1.0 * 1.0 + 1.0 * 0.2)


def test_pd_derivative_responds_to_error_change():
    """PD controller derivative term responds to change in error."""
    pid = PIDController(kp=0.0, kd=1.0)
    pid.compute(error=0.0, dt=0.1)
    result = pid.compute(error=1.0, dt=0.1)
    # derivative = (1.0 - 0.0) / 0.1 = 10.0; output = 0.0 + 0.0 + 1.0 * 10.0 = 10.0
    assert result == pytest.approx(10.0)


def test_full_pid():
    """Full PID sums proportional, integral, and derivative contributions."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    # First call: no derivative (prev_error is None), integral = 1.0 * 0.5 = 0.5
    pid.compute(error=1.0, dt=0.5)
    # Second call: error=2.0, integral = 0.5 + 2.0*0.5 = 1.5
    # derivative = (2.0 - 1.0) / 0.5 = 2.0
    # output = 1.0*2.0 + 1.0*1.5 + 1.0*2.0 = 5.5
    result = pid.compute(error=2.0, dt=0.5)
    assert result == pytest.approx(5.5)


def test_anti_windup_clamps_integral():
    """Integral is clamped to integral_max regardless of accumulated error."""
    pid = PIDController(kp=0.0, ki=1.0, integral_max=1.0)
    pid.compute(error=100.0, dt=1.0)
    pid.compute(error=100.0, dt=1.0)
    result = pid.compute(error=0.0, dt=1.0)
    # ki * integral should be clamped to 1.0 * 1.0 = 1.0
    assert result == pytest.approx(1.0)


def test_output_clamped_to_max():
    """Output is clamped to output_max when control law exceeds the bound."""
    pid = PIDController(kp=10.0, output_min=-5.0, output_max=5.0)
    result = pid.compute(error=100.0, dt=0.1)
    assert result == pytest.approx(5.0)


def test_output_clamped_to_min():
    """Output is clamped to output_min when control law is below the bound."""
    pid = PIDController(kp=10.0, output_min=-5.0, output_max=5.0)
    result = pid.compute(error=-100.0, dt=0.1)
    assert result == pytest.approx(-5.0)


def test_reset_clears_state():
    """Reset clears integral and previous error so next compute starts fresh."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    pid.compute(error=5.0, dt=0.1)
    pid.reset()
    # After reset: integral=0, prev_error=None; output = kp*error only
    result = pid.compute(error=2.0, dt=0.1)
    assert result == pytest.approx(1.0 * 2.0 + 1.0 * (2.0 * 0.1) + 0.0)


def test_zero_dt_returns_zero():
    """Zero dt returns 0.0 without modifying internal state."""
    pid = PIDController(kp=5.0)
    result = pid.compute(error=1.0, dt=0.0)
    assert result == pytest.approx(0.0)


def test_negative_dt_returns_zero():
    """Negative dt returns 0.0 without modifying internal state."""
    pid = PIDController(kp=5.0)
    result = pid.compute(error=1.0, dt=-0.1)
    assert result == pytest.approx(0.0)
