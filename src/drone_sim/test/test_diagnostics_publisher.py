"""Tests for drone diagnostics publisher logic."""

import pytest


def battery_level(pct: float, warn: float, error: float) -> int:
    """Replicate the battery level logic from DroneDiagnosticsPublisher."""
    if pct <= error:
        return 2  # ERROR
    elif pct <= warn:
        return 1  # WARN
    return 0  # OK


class TestBatteryDiagnostics:
    def test_ok(self):
        assert battery_level(80.0, 30.0, 15.0) == 0

    def test_warn(self):
        assert battery_level(25.0, 30.0, 15.0) == 1

    def test_error(self):
        assert battery_level(10.0, 30.0, 15.0) == 2

    def test_at_warn_threshold(self):
        assert battery_level(30.0, 30.0, 15.0) == 1

    def test_at_error_threshold(self):
        assert battery_level(15.0, 30.0, 15.0) == 2

    def test_zero_battery(self):
        assert battery_level(0.0, 30.0, 15.0) == 2

    def test_full_battery(self):
        assert battery_level(100.0, 30.0, 15.0) == 0
