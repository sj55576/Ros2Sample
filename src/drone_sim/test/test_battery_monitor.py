"""Tests for battery monitor drain calculations."""

import pytest


def compute_throttle(vx: float, vy: float, vz: float, wz: float) -> float:
    """Replicate throttle calculation from BatteryMonitor._on_cmd_vel."""
    return min(1.0, (abs(vx) + abs(vy) + abs(vz) + abs(wz)) / 4.0)


def compute_drain(
    remaining_wh: float,
    capacity_wh: float,
    idle_power_w: float,
    motor_power_w: float,
    throttle: float,
    dt_sec: float,
) -> tuple:
    """Replicate drain calculation from BatteryMonitor._tick.

    Returns (new_remaining_wh, percentage, voltage, current).
    """
    power_w = idle_power_w + throttle * motor_power_w
    drain_wh = power_w * (dt_sec / 3600.0)
    new_remaining = max(0.0, remaining_wh - drain_wh)
    pct = new_remaining / capacity_wh if capacity_wh > 0.0 else 0.0
    voltage = 10.0 + pct * 2.6
    current = -(power_w / voltage) if voltage > 0.0 else 0.0
    return (new_remaining, pct, voltage, current)


class TestThrottle:
    def test_zero_velocity(self):
        assert compute_throttle(0.0, 0.0, 0.0, 0.0) == 0.0

    def test_max_throttle_clamped(self):
        assert compute_throttle(5.0, 5.0, 5.0, 5.0) == 1.0

    def test_partial_throttle(self):
        result = compute_throttle(1.0, 0.0, 0.0, 0.0)
        assert result == pytest.approx(0.25)

    def test_all_axes(self):
        result = compute_throttle(1.0, 1.0, 1.0, 1.0)
        assert result == pytest.approx(1.0)

    def test_negative_velocity_uses_abs(self):
        result = compute_throttle(-2.0, 0.0, 0.0, 0.0)
        assert result == pytest.approx(0.5)


class TestDrain:
    def test_idle_drain(self):
        remaining, pct, voltage, current = compute_drain(
            remaining_wh=50.0, capacity_wh=50.0,
            idle_power_w=5.0, motor_power_w=80.0,
            throttle=0.0, dt_sec=1.0,
        )
        expected_drain = 5.0 / 3600.0
        assert remaining == pytest.approx(50.0 - expected_drain, abs=1e-6)
        assert pct == pytest.approx(remaining / 50.0)
        assert voltage > 10.0

    def test_full_throttle_drain(self):
        remaining, pct, _, _ = compute_drain(
            remaining_wh=50.0, capacity_wh=50.0,
            idle_power_w=5.0, motor_power_w=80.0,
            throttle=1.0, dt_sec=1.0,
        )
        expected_drain = 85.0 / 3600.0
        assert remaining == pytest.approx(50.0 - expected_drain, abs=1e-6)

    def test_drain_does_not_go_negative(self):
        remaining, pct, _, _ = compute_drain(
            remaining_wh=0.001, capacity_wh=50.0,
            idle_power_w=5.0, motor_power_w=80.0,
            throttle=1.0, dt_sec=3600.0,
        )
        assert remaining == 0.0
        assert pct == 0.0

    def test_voltage_range(self):
        _, _, v_full, _ = compute_drain(50.0, 50.0, 5.0, 0.0, 0.0, 0.0)
        assert v_full == pytest.approx(12.6, abs=0.1)

        _, _, v_empty, _ = compute_drain(0.0, 50.0, 5.0, 0.0, 0.0, 1.0)
        assert v_empty == pytest.approx(10.0, abs=0.1)

    def test_current_is_negative(self):
        _, _, _, current = compute_drain(
            50.0, 50.0, 5.0, 80.0, 0.5, 1.0,
        )
        assert current < 0.0

    def test_zero_capacity(self):
        _, pct, _, _ = compute_drain(
            0.0, 0.0, 5.0, 80.0, 0.0, 1.0,
        )
        assert pct == 0.0

    def test_zero_dt(self):
        remaining, _, _, _ = compute_drain(
            50.0, 50.0, 5.0, 80.0, 1.0, 0.0,
        )
        assert remaining == 50.0
