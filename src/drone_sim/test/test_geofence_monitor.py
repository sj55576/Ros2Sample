"""Tests for the drone_sim.geofence_monitor module."""

import pytest

from drone_sim.geofence_utils import check_boundary

BOUNDS_MIN = (-10.0, -10.0, 0.0)
BOUNDS_MAX = (10.0, 10.0, 20.0)
MARGIN = 1.0


def test_position_safely_inside():
    """Center of the bounding box is 'safe' and position is unchanged."""
    status, pos = check_boundary(0.0, 0.0, 10.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'safe'
    assert pos[0] == pytest.approx(0.0)
    assert pos[1] == pytest.approx(0.0)
    assert pos[2] == pytest.approx(10.0)


def test_position_in_warning_zone():
    """Position within margin of the boundary but not outside is 'warning'."""
    status, _ = check_boundary(9.5, 0.0, 10.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'warning'


def test_position_outside_boundary():
    """Position past the hard boundary is 'breach'."""
    status, _ = check_boundary(11.0, 0.0, 10.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'


def test_clamped_position_on_breach():
    """Breached position is clamped to bounds_min+margin / bounds_max-margin."""
    status, pos = check_boundary(15.0, -15.0, 25.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'
    assert pos[0] == pytest.approx(BOUNDS_MAX[0] - MARGIN)
    assert pos[1] == pytest.approx(BOUNDS_MIN[1] + MARGIN)
    assert pos[2] == pytest.approx(BOUNDS_MAX[2] - MARGIN)


def test_z_boundary_breach():
    """Z axis breach (above ceiling) is detected independently."""
    status, pos = check_boundary(0.0, 0.0, 21.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'
    assert pos[2] == pytest.approx(BOUNDS_MAX[2] - MARGIN)


def test_exactly_on_boundary():
    """Position exactly on the hard boundary edge is 'breach'."""
    status, _ = check_boundary(10.0, 0.0, 10.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'


def test_exactly_at_margin():
    """Position exactly at the margin edge (but inside boundary) is 'warning'."""
    safe_max_x = BOUNDS_MAX[0] - MARGIN
    status, _ = check_boundary(safe_max_x, 0.0, 10.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'warning'


def test_all_axes_breached():
    """All three axes outside the boundary simultaneously yields 'breach'."""
    status, pos = check_boundary(-12.0, 12.0, -1.0, BOUNDS_MIN, BOUNDS_MAX, MARGIN)
    assert status == 'breach'
    assert pos[0] == pytest.approx(BOUNDS_MIN[0] + MARGIN)
    assert pos[1] == pytest.approx(BOUNDS_MAX[1] - MARGIN)
    assert pos[2] == pytest.approx(BOUNDS_MIN[2] + MARGIN)
