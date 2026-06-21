"""Tests for the ground_robot_sim.waypoint_follower module."""

import pytest

from ground_robot_sim.waypoint_follower import parse_waypoints_xy


def test_parse_waypoints_xy_valid_pairs():
    """Valid waypoint pairs are parsed into ordered (x, y) tuples."""
    result = parse_waypoints_xy([1.0, 2.0, 3.5, 4.5])
    assert result == [(1.0, 2.0), (3.5, 4.5)]


def test_parse_waypoints_xy_empty_list_raises():
    """An empty waypoint list raises ValueError."""
    with pytest.raises(ValueError, match='waypoints must be a non-empty flat list'):
        parse_waypoints_xy([])


def test_parse_waypoints_xy_odd_length_raises():
    """A waypoint list with an odd number of values raises ValueError."""
    with pytest.raises(ValueError, match='waypoints must be a non-empty flat list'):
        parse_waypoints_xy([1.0, 2.0, 3.0])


def test_parse_waypoints_xy_single_pair():
    """A single waypoint pair is accepted."""
    result = parse_waypoints_xy([5.0, -1.25])
    assert result == [(5.0, -1.25)]


def test_parse_waypoints_xy_converts_values_to_float():
    """Integer waypoint inputs are converted to float outputs."""
    result = parse_waypoints_xy([1, 2, 3, 4])
    assert result == [(1.0, 2.0), (3.0, 4.0)]
    assert all(isinstance(value, float) for pair in result for value in pair)


def test_parse_waypoints_xy_rejects_non_finite_values():
    """NaN or infinite waypoint components are rejected before control starts."""
    with pytest.raises(ValueError, match='finite values'):
        parse_waypoints_xy([0.0, float('inf')])
