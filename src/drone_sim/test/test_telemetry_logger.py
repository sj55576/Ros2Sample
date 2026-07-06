"""Tests for the drone_sim.telemetry_logger module."""

import json
import math

from drone_sim.telemetry_utils import compute_distance, compute_speed, format_telemetry
import pytest


def test_compute_distance_same_point():
    """Distance from a point to itself is 0.0."""
    assert compute_distance(1.0, 2.0, 3.0, 1.0, 2.0, 3.0) == pytest.approx(0.0)


def test_compute_distance_unit_x():
    """Distance from origin to (1,0,0) is 1.0."""
    assert compute_distance(0.0, 0.0, 0.0, 1.0, 0.0, 0.0) == pytest.approx(1.0)


def test_compute_distance_3d_diagonal():
    """Distance from origin to (1,1,1) is sqrt(3)."""
    assert compute_distance(0.0, 0.0, 0.0, 1.0, 1.0, 1.0) == pytest.approx(math.sqrt(3))


def test_compute_speed_zero():
    """All-zero velocity components give a scalar speed of 0.0."""
    assert compute_speed(0.0, 0.0, 0.0) == pytest.approx(0.0)


def test_compute_speed_single_axis():
    """Velocity of (3,0,0) gives a scalar speed of 3.0."""
    assert compute_speed(3.0, 0.0, 0.0) == pytest.approx(3.0)


def test_compute_speed_3d():
    """Velocity of (1,2,2) gives a scalar speed of 3.0."""
    assert compute_speed(1.0, 2.0, 2.0) == pytest.approx(3.0)


def test_format_telemetry_returns_valid_json():
    """format_telemetry output can be parsed as valid JSON with correct field values."""
    result = format_telemetry(
        total_distance_m=100.0,
        max_altitude_m=50.0,
        max_speed_ms=15.0,
        battery_pct=80.0,
        flight_time_sec=120.0,
        current_x=1.0,
        current_y=2.0,
        current_z=3.0,
    )
    data = json.loads(result)
    assert data['total_distance_m'] == pytest.approx(100.0)
    assert data['max_altitude_m'] == pytest.approx(50.0)
    assert data['max_speed_ms'] == pytest.approx(15.0)
    assert data['battery_pct'] == pytest.approx(80.0)
    assert data['flight_time_sec'] == pytest.approx(120.0)
    assert data['current_x'] == pytest.approx(1.0)
    assert data['current_y'] == pytest.approx(2.0)
    assert data['current_z'] == pytest.approx(3.0)


def test_format_telemetry_contains_all_fields():
    """format_telemetry output contains all 8 expected keys."""
    result = format_telemetry(
        total_distance_m=0.0,
        max_altitude_m=0.0,
        max_speed_ms=0.0,
        battery_pct=100.0,
        flight_time_sec=0.0,
        current_x=0.0,
        current_y=0.0,
        current_z=0.0,
    )
    data = json.loads(result)
    expected_keys = {
        'total_distance_m',
        'max_altitude_m',
        'max_speed_ms',
        'battery_pct',
        'flight_time_sec',
        'current_x',
        'current_y',
        'current_z',
    }
    assert expected_keys == set(data.keys())
