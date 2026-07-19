"""Tests for the drone_sim.telemetry_utils module."""

import json
import math

from drone_sim.telemetry_utils import (
    compute_distance,
    compute_speed,
    format_telemetry,
)
import pytest


def test_compute_distance_same_point_is_zero():
    """Distance between identical points is zero."""
    assert compute_distance(1.0, 2.0, 3.0, 1.0, 2.0, 3.0) == pytest.approx(0.0)


def test_compute_distance_along_single_axis():
    """Distance along a single axis equals the absolute coordinate difference."""
    assert compute_distance(0.0, 0.0, 0.0, 3.0, 0.0, 0.0) == pytest.approx(3.0)


def test_compute_distance_3_4_5_triangle():
    """A 3-4-0 offset gives the classic 5 unit distance."""
    assert compute_distance(0.0, 0.0, 0.0, 3.0, 4.0, 0.0) == pytest.approx(5.0)


def test_compute_distance_is_symmetric():
    """Distance is the same regardless of point order."""
    forward = compute_distance(1.0, 2.0, 3.0, 4.0, 6.0, 3.0)
    backward = compute_distance(4.0, 6.0, 3.0, 1.0, 2.0, 3.0)
    assert forward == pytest.approx(backward)


def test_compute_speed_zero_velocity():
    """Zero velocity components give zero speed."""
    assert compute_speed(0.0, 0.0, 0.0) == pytest.approx(0.0)


def test_compute_speed_single_axis():
    """Speed with motion along a single axis equals that axis's magnitude."""
    assert compute_speed(-4.0, 0.0, 0.0) == pytest.approx(4.0)


def test_compute_speed_combined_axes():
    """Speed combines all three velocity components in quadrature."""
    assert compute_speed(1.0, 2.0, 2.0) == pytest.approx(3.0)


def test_compute_speed_matches_sqrt_of_squares():
    """Speed equals the square root of the sum of squares for arbitrary input."""
    vx, vy, vz = 1.5, -2.5, 0.5
    expected = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    assert compute_speed(vx, vy, vz) == pytest.approx(expected)


def test_format_telemetry_returns_valid_json():
    """Formatted telemetry parses back into a dict with the correct keys."""
    result = format_telemetry(
        total_distance_m=100.0,
        max_altitude_m=50.0,
        max_speed_ms=12.5,
        battery_pct=75.0,
        flight_time_sec=300.0,
        current_x=1.0,
        current_y=2.0,
        current_z=3.0,
    )
    parsed = json.loads(result)
    assert parsed == {
        'total_distance_m': 100.0,
        'max_altitude_m': 50.0,
        'max_speed_ms': 12.5,
        'battery_pct': 75.0,
        'flight_time_sec': 300.0,
        'current_x': 1.0,
        'current_y': 2.0,
        'current_z': 3.0,
    }


def test_format_telemetry_handles_zero_values():
    """Formatting all-zero telemetry values still produces valid JSON."""
    result = format_telemetry(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    parsed = json.loads(result)
    assert all(value == 0.0 for value in parsed.values())


def test_format_telemetry_handles_negative_coordinates():
    """Negative current position coordinates round-trip correctly through JSON."""
    result = format_telemetry(1.0, 2.0, 3.0, 50.0, 10.0, -1.0, -2.0, -3.0)
    parsed = json.loads(result)
    assert parsed['current_x'] == -1.0
    assert parsed['current_y'] == -2.0
    assert parsed['current_z'] == -3.0
