"""Pure kinematics helpers for a planar 2-DOF manipulator."""

import math
from typing import List, Sequence, Tuple


Point2 = Tuple[float, float]
JointPair = Tuple[float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a floating-point value into [lower, upper]."""
    return max(lower, min(upper, value))


def parse_targets_xy(raw_targets: Sequence[float]) -> List[Point2]:
    """Parse a flat [x1, y1, x2, y2, ...] sequence into (x, y) tuples."""
    if len(raw_targets) < 2 or len(raw_targets) % 2 != 0:
        raise ValueError('targets_xy must contain one or more (x, y) pairs')
    return [
        (float(raw_targets[index]), float(raw_targets[index + 1]))
        for index in range(0, len(raw_targets), 2)
    ]


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to the [-pi, pi] range."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def forward_kinematics(theta1: float, theta2: float, l1: float, l2: float) -> Point2:
    """Compute the end-effector x,y position from two joint angles and link lengths."""
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
    return (x, y)


def inverse_kinematics(
    x: float,
    y: float,
    l1: float,
    l2: float,
    elbow_up: bool = False,
) -> JointPair:
    """Solve planar 2-link inverse kinematics.

    Raises ValueError when the target is outside the reachable annulus.
    """
    radius = math.hypot(x, y)
    if radius > l1 + l2 or radius < abs(l1 - l2):
        raise ValueError(f'target ({x:.3f}, {y:.3f}) is unreachable')

    c2 = clamp((x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0)
    s2_abs = math.sqrt(max(0.0, 1.0 - c2 * c2))
    s2 = -s2_abs if elbow_up else s2_abs

    theta2 = math.atan2(s2, c2)
    k1 = l1 + l2 * c2
    k2 = l2 * s2
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return (wrap_angle(theta1), wrap_angle(theta2))


def step_towards(current: float, target: float, max_delta: float) -> float:
    """Move a scalar toward target by up to max_delta."""
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)
