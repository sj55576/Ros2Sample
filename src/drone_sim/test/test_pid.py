"""Tests for the drone_sim.pid module."""

from drone_sim.pid import PIDController
import pytest


def test_p_only_output():
    """P-only controller output equals kp multiplied by error."""
    pid = PIDController(kp=2.0)
    assert pid.compute(3.0, 0.1) == pytest.approx(6.0)


def test_p_only_no_integral_accumulation():
    """P-only controller produces no integral effect across multiple calls."""
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    pid.compute(1.0, 0.1)
    pid.compute(1.0, 0.1)
    result = pid.compute(1.0, 0.1)
    assert result == pytest.approx(1.0)


def test_pi_integral_accumulates():
    """PI controller integral term grows with each call at constant error."""
    pid = PIDController(kp=0.0, ki=1.0)
    pid.compute(1.0, 0.1)  # integral = 0.1
    result = pid.compute(1.0, 0.1)  # integral = 0.2
    assert result == pytest.approx(0.2)


def test_pi_integral_three_steps():
    """PI controller integral sums correctly over three equal timesteps."""
    pid = PIDController(kp=0.0, ki=2.0)
    pid.compute(1.0, 0.5)  # integral = 0.5
    pid.compute(1.0, 0.5)  # integral = 1.0
    result = pid.compute(1.0, 0.5)  # integral = 1.5, output = 2.0 * 1.5 = 3.0
    assert result == pytest.approx(3.0)


def test_pd_derivative_on_first_call():
    """PD controller derivative is zero on the first call (no previous error)."""
    pid = PIDController(kp=0.0, kd=1.0)
    result = pid.compute(5.0, 0.1)
    assert result == pytest.approx(0.0)


def test_pd_derivative_on_second_call():
    """PD controller derivative responds to error change on the second call."""
    pid = PIDController(kp=0.0, kd=1.0)
    pid.compute(2.0, 0.1)
    # derivative = (4.0 - 2.0) / 0.1 = 20.0
    result = pid.compute(4.0, 0.1)
    assert result == pytest.approx(20.0)


def test_pd_derivative_decreasing_error():
    """PD controller produces negative derivative when error decreases."""
    pid = PIDController(kp=0.0, kd=1.0)
    pid.compute(4.0, 0.1)
    # derivative = (2.0 - 4.0) / 0.1 = -20.0
    result = pid.compute(2.0, 0.1)
    assert result == pytest.approx(-20.0)


def test_full_pid_combines_all_terms():
    """Full PID output sums proportional, integral, and derivative correctly."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    pid.compute(1.0, 1.0)  # P=1, I=1, D=0 -> output=2
    # Second call: error=2, dt=1 -> P=2, I=1+2=3, D=(2-1)/1=1 -> output=6
    result = pid.compute(2.0, 1.0)
    assert result == pytest.approx(6.0)


def test_anti_windup_clamps_integral_positive():
    """Integral is clamped to integral_max when it would exceed the limit."""
    pid = PIDController(kp=0.0, ki=1.0, integral_max=0.5)
    pid.compute(1.0, 1.0)  # integral would be 1.0, clamped to 0.5
    result = pid.compute(0.0, 1.0)  # P=0, I=0.5*1.0=0.5, D=0
    assert result == pytest.approx(0.5)


def test_anti_windup_clamps_integral_negative():
    """Integral is clamped to -integral_max when error is persistently negative."""
    pid = PIDController(kp=0.0, ki=1.0, integral_max=0.5)
    pid.compute(-1.0, 1.0)  # integral clamped to -0.5
    result = pid.compute(0.0, 1.0)  # output = ki * integral = 1.0 * -0.5 = -0.5
    assert result == pytest.approx(-0.5)


def test_output_clamped_to_max():
    """Output is clamped to output_max when the raw value exceeds it."""
    pid = PIDController(kp=10.0, output_max=5.0)
    result = pid.compute(3.0, 0.1)  # raw = 30.0, clamped to 5.0
    assert result == pytest.approx(5.0)


def test_output_clamped_to_min():
    """Output is clamped to output_min when the raw value is below it."""
    pid = PIDController(kp=10.0, output_min=-5.0)
    result = pid.compute(-3.0, 0.1)  # raw = -30.0, clamped to -5.0
    assert result == pytest.approx(-5.0)


def test_output_clamped_symmetric():
    """Symmetric output bounds are respected for both positive and negative errors."""
    pid = PIDController(kp=10.0, output_min=-2.0, output_max=2.0)
    assert pid.compute(1.0, 0.1) == pytest.approx(2.0)
    pid.reset()
    assert pid.compute(-1.0, 0.1) == pytest.approx(-2.0)


def test_reset_clears_integral():
    """Calling reset() zeroes the accumulated integral."""
    pid = PIDController(kp=0.0, ki=1.0)
    pid.compute(1.0, 1.0)  # integral = 1.0
    pid.reset()
    result = pid.compute(1.0, 1.0)  # integral starts fresh: 1.0, output = 1.0
    assert result == pytest.approx(1.0)


def test_reset_clears_derivative_state():
    """Calling reset() removes previous error so derivative is zero on next call."""
    pid = PIDController(kp=0.0, kd=1.0)
    pid.compute(5.0, 0.1)  # prev_error = 5.0
    pid.reset()
    result = pid.compute(5.0, 0.1)  # no prev_error -> derivative = 0
    assert result == pytest.approx(0.0)


def test_zero_dt_returns_zero():
    """A timestep of zero returns 0.0 without updating state."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    result = pid.compute(5.0, 0.0)
    assert result == pytest.approx(0.0)


def test_negative_dt_returns_zero():
    """A negative timestep returns 0.0 without updating state."""
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    result = pid.compute(5.0, -0.1)
    assert result == pytest.approx(0.0)


def test_zero_dt_does_not_change_integral():
    """State is not modified when dt is zero, so the next call is unaffected."""
    pid = PIDController(kp=0.0, ki=1.0)
    pid.compute(5.0, 0.0)  # should not accumulate
    result = pid.compute(1.0, 1.0)  # integral = 1.0, output = 1.0
    assert result == pytest.approx(1.0)
