"""Tests for altitude hold PID control logic."""

import pytest
from drone_sim.pid import PIDController


def altitude_hold_command(
    target: float, current: float, pid: PIDController, dt: float,
) -> float:
    """Replicate AltitudeHold._publish_command logic."""
    error = target - current
    return pid.compute(error, dt)


class TestAltitudeHold:
    def test_at_target(self):
        pid = PIDController(kp=1.3, ki=0.1, kd=0.3, output_min=-1.5, output_max=1.5)
        cmd = altitude_hold_command(2.0, 2.0, pid, 0.05)
        assert cmd == pytest.approx(0.0, abs=0.01)

    def test_below_target_commands_up(self):
        pid = PIDController(kp=1.3, ki=0.1, kd=0.3, output_min=-1.5, output_max=1.5)
        cmd = altitude_hold_command(2.0, 1.0, pid, 0.05)
        assert cmd > 0.0

    def test_above_target_commands_down(self):
        pid = PIDController(kp=1.3, ki=0.1, kd=0.3, output_min=-1.5, output_max=1.5)
        cmd = altitude_hold_command(2.0, 3.0, pid, 0.05)
        assert cmd < 0.0

    def test_output_clamped(self):
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_min=-1.5, output_max=1.5)
        cmd = altitude_hold_command(100.0, 0.0, pid, 0.05)
        assert cmd == pytest.approx(1.5)

    def test_convergence(self):
        """Simulate several steps and verify the altitude converges."""
        pid = PIDController(kp=1.3, ki=0.1, kd=0.3, output_min=-1.5, output_max=1.5)
        target = 2.0
        current = 0.0
        dt = 0.05
        for _ in range(200):
            error = target - current
            cmd = pid.compute(error, dt)
            current += cmd * dt
        assert current == pytest.approx(target, abs=0.1)

    def test_integral_helps_steady_state(self):
        """With disturbance (bias), integral term helps reach target."""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.1, output_min=-2.0, output_max=2.0)
        target = 2.0
        current = 0.0
        bias = -0.3
        dt = 0.05
        for _ in range(400):
            error = target - current
            cmd = pid.compute(error, dt)
            current += (cmd + bias) * dt
        assert current == pytest.approx(target, abs=0.3)

    def test_zero_dt_returns_zero(self):
        pid = PIDController(kp=1.3, ki=0.1, kd=0.3)
        cmd = altitude_hold_command(2.0, 0.0, pid, 0.0)
        assert cmd == 0.0
