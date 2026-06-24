"""Tests for ground robot diagnostics publisher logic."""

import math
import pytest


def proximity_level(min_range: float, warn_threshold: float) -> int:
    """Replicate proximity level logic from GroundRobotDiagnosticsPublisher."""
    if min_range < warn_threshold:
        return 1  # WARN
    return 0  # OK


def speed_level(speed: float, warn_threshold: float) -> int:
    """Replicate speed level logic."""
    if speed > warn_threshold:
        return 1  # WARN
    return 0  # OK


class TestProximityDiagnostics:
    def test_clear(self):
        assert proximity_level(2.0, 0.3) == 0

    def test_warn(self):
        assert proximity_level(0.2, 0.3) == 1

    def test_at_threshold(self):
        assert proximity_level(0.3, 0.3) == 0

    def test_very_close(self):
        assert proximity_level(0.01, 0.3) == 1


class TestSpeedDiagnostics:
    def test_normal(self):
        assert speed_level(0.3, 0.7) == 0

    def test_warn(self):
        assert speed_level(0.8, 0.7) == 1

    def test_at_threshold(self):
        assert speed_level(0.7, 0.7) == 0

    def test_stopped(self):
        assert speed_level(0.0, 0.7) == 0
