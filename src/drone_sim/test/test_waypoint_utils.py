"""Tests for the drone_sim.waypoint_utils module."""

import pytest

from drone_sim.waypoint_utils import parse_waypoints


def test_parse_waypoints_valid_flat_list():
    """A flat list of multiples of 3 parses into the correct triples."""
    raw = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    result = parse_waypoints(raw)
    assert result == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]


def test_parse_waypoints_length_not_multiple_of_three():
    """A list whose length is not a multiple of 3 raises ValueError."""
    with pytest.raises(ValueError, match='waypoints parameter must contain x, y, z triples'):
        parse_waypoints([1.0, 2.0, 3.0, 4.0])


def test_parse_waypoints_fewer_than_three():
    """A list with fewer than 3 values raises ValueError."""
    with pytest.raises(ValueError, match='waypoints parameter must contain x, y, z triples'):
        parse_waypoints([1.0, 2.0])


def test_parse_waypoints_values_are_floats():
    """Parsed waypoint components are Python floats."""
    result = parse_waypoints([1, 2, 3])
    assert all(isinstance(v, float) for triple in result for v in triple)


def test_parse_waypoints_rejects_non_finite_values():
    """NaN or infinite waypoint components are rejected before node startup."""
    with pytest.raises(ValueError, match='finite values'):
        parse_waypoints([0.0, float('nan'), 1.0])
