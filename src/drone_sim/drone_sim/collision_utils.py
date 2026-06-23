"""Pure-function utilities for drone collision avoidance using potential fields."""

import math
from typing import List, Tuple

Pos3 = Tuple[float, float, float]


def compute_repulsive_force(
    position: Pos3,
    others: List[Pos3],
    safety_distance: float,
    influence_distance: float,
    gain: float,
) -> Tuple[float, float, float]:
    """Compute the total repulsive force on a drone from all neighbours.

    Uses an inverse-square potential field: for each neighbour within
    *influence_distance*, the repulsive magnitude is
        gain * (1/dist - 1/influence_distance) * (1/dist^2)
    in the direction away from the neighbour.  Neighbours closer than
    *safety_distance* are clamped to *safety_distance* to avoid singularity.

    Returns (fx, fy, fz) total repulsive force vector.
    """
    fx, fy, fz = 0.0, 0.0, 0.0
    for other in others:
        dx = position[0] - other[0]
        dy = position[1] - other[1]
        dz = position[2] - other[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 1e-9 or dist >= influence_distance:
            continue
        # unit vector from other to self (computed from original distance)
        inv_dist = 1.0 / dist
        ux = dx * inv_dist
        uy = dy * inv_dist
        uz = dz * inv_dist
        # clamp dist to safety_distance to avoid singularity
        dist = max(dist, safety_distance)
        inv_d = 1.0 / dist
        inv_inf = 1.0 / influence_distance
        magnitude = gain * (inv_d - inv_inf) * (inv_d * inv_d)
        fx += magnitude * ux
        fy += magnitude * uy
        fz += magnitude * uz
    return (fx, fy, fz)


def apply_avoidance(
    target: Pos3,
    repulsive: Tuple[float, float, float],
    max_adjustment: float,
) -> Pos3:
    """Offset *target* by *repulsive* force, clamped to *max_adjustment* magnitude."""
    ax, ay, az = repulsive
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag > max_adjustment and mag > 1e-9:
        scale = max_adjustment / mag
        ax *= scale
        ay *= scale
        az *= scale
    return (target[0] + ax, target[1] + ay, target[2] + az)
