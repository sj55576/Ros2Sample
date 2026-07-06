"""Tests for the drone_sim.collision_utils module."""

import math

from drone_sim.collision_utils import apply_avoidance, compute_repulsive_force
import pytest


def test_no_neighbours():
    """Empty neighbour list returns zero force."""
    result = compute_repulsive_force((0.0, 0.0, 0.0), [], 0.5, 3.0, 1.5)
    assert result == pytest.approx((0.0, 0.0, 0.0))


def test_out_of_influence():
    """Neighbour beyond influence_distance contributes no force."""
    result = compute_repulsive_force(
        (0.0, 0.0, 0.0), [(5.0, 0.0, 0.0)], 0.5, 3.0, 1.5
    )
    assert result == pytest.approx((0.0, 0.0, 0.0))


def test_single_neighbour_repulsion():
    """Force from a single neighbour points away from it."""
    # Neighbour is at (-1, 0, 0), own position at (0, 0, 0) -> force should point in +x
    fx, fy, fz = compute_repulsive_force(
        (0.0, 0.0, 0.0), [(-1.0, 0.0, 0.0)], 0.5, 3.0, 1.5
    )
    assert fx > 0.0
    assert fy == pytest.approx(0.0, abs=1e-9)
    assert fz == pytest.approx(0.0, abs=1e-9)


def test_force_increases_with_proximity():
    """A closer neighbour produces a larger repulsive force magnitude."""
    def force_mag(dist: float) -> float:
        fx, fy, fz = compute_repulsive_force(
            (0.0, 0.0, 0.0), [(dist, 0.0, 0.0)], 0.1, 5.0, 1.0
        )
        return math.sqrt(fx * fx + fy * fy + fz * fz)

    assert force_mag(0.5) > force_mag(1.0)
    assert force_mag(1.0) > force_mag(2.0)


def test_safety_distance_clamp():
    """Neighbour at distance zero does not cause division by zero."""
    result = compute_repulsive_force(
        (0.0, 0.0, 0.0), [(0.0, 0.0, 0.0)], 0.5, 3.0, 1.5
    )
    assert all(math.isfinite(v) for v in result)


def test_apply_avoidance_no_force():
    """Zero repulsive force returns the original target unchanged."""
    target = (1.0, 2.0, 3.0)
    result = apply_avoidance(target, (0.0, 0.0, 0.0), max_adjustment=2.0)
    assert result == pytest.approx(target)


def test_apply_avoidance_clamped():
    """A large force vector is clamped to max_adjustment magnitude."""
    target = (0.0, 0.0, 0.0)
    max_adj = 2.0
    result = apply_avoidance(target, (10.0, 0.0, 0.0), max_adjustment=max_adj)
    offset_mag = math.sqrt(sum((r - t) ** 2 for r, t in zip(result, target)))
    assert offset_mag == pytest.approx(max_adj)


def test_apply_avoidance_small_force():
    """A force smaller than max_adjustment is applied directly without clamping."""
    target = (1.0, 1.0, 1.0)
    repulsive = (0.3, 0.4, 0.0)
    result = apply_avoidance(target, repulsive, max_adjustment=5.0)
    assert result == pytest.approx((1.3, 1.4, 1.0))


def test_symmetry():
    """Two drones equidistant on opposite sides produce cancelling forces."""
    fx, fy, fz = compute_repulsive_force(
        (0.0, 0.0, 0.0),
        [(1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)],
        0.1,
        5.0,
        1.0,
    )
    assert fx == pytest.approx(0.0, abs=1e-9)
    assert fy == pytest.approx(0.0, abs=1e-9)
    assert fz == pytest.approx(0.0, abs=1e-9)


def test_3d_repulsion():
    """Repulsive force z-component is non-zero when neighbour differs in z only."""
    fx, fy, fz = compute_repulsive_force(
        (0.0, 0.0, 0.0), [(0.0, 0.0, -1.0)], 0.5, 3.0, 1.5
    )
    assert fx == pytest.approx(0.0, abs=1e-9)
    assert fy == pytest.approx(0.0, abs=1e-9)
    assert fz > 0.0
