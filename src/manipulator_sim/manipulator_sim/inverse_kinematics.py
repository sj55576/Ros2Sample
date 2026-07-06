"""Analytical inverse kinematics for a 2-link planar manipulator."""

import math
from typing import Optional, Tuple


def solve_ik(
    x: float,
    y: float,
    l1: float,
    l2: float,
    elbow_up: bool = True,
) -> Optional[Tuple[float, float]]:
    """
    Compute joint angles (theta1, theta2) to reach target (x, y).

    Returns None if the target is unreachable (outside the workspace).
    """
    dist_sq = x * x + y * y
    dist = math.sqrt(dist_sq)
    reach_max = l1 + l2
    reach_min = abs(l1 - l2)
    if dist > reach_max or dist < reach_min:
        return None
    cos_q2 = (dist_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    if elbow_up:
        q2 = -math.acos(cos_q2)
    else:
        q2 = math.acos(cos_q2)
    q1 = math.atan2(y, x) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    return (q1, q2)


def workspace_radius(l1: float, l2: float) -> Tuple[float, float]:
    """Return (min_reach, max_reach) of the 2-link arm."""
    return (abs(l1 - l2), l1 + l2)


def nearest_reachable(
    x: float,
    y: float,
    l1: float,
    l2: float,
) -> Tuple[float, float]:
    """Project an unreachable target to the nearest point on the workspace boundary."""
    dist = math.sqrt(x * x + y * y)
    if dist < 1e-9:
        return (l1 + l2, 0.0)
    r_min, r_max = workspace_radius(l1, l2)
    if dist > r_max:
        scale = r_max / dist
    elif dist < r_min:
        scale = r_min / dist
    else:
        return (x, y)
    return (x * scale, y * scale)
