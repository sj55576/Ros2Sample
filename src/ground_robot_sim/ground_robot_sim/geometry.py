"""Pure geometry helpers for the ground-robot simulation (no ROS imports)."""

import math
from typing import List, Sequence, Tuple


Circle = Tuple[float, float, float]


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion for a planar yaw angle."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def ray_circle_distance(
    origin_x: float,
    origin_y: float,
    ray_x: float,
    ray_y: float,
    circle: Circle,
) -> float:
    """Return the nearest forward intersection distance for a ray and circle."""
    center_x, center_y, radius = circle
    offset_x = origin_x - center_x
    offset_y = origin_y - center_y
    projection = offset_x * ray_x + offset_y * ray_y
    constant = offset_x * offset_x + offset_y * offset_y - radius * radius
    discriminant = projection * projection - constant
    if discriminant < 0.0:
        return math.inf

    root = math.sqrt(discriminant)
    first = -projection - root
    second = -projection + root
    if first >= 0.0:
        return first
    if second >= 0.0:
        return second
    return math.inf


def parse_circles(raw_values: Sequence[float]) -> List[Circle]:
    """Parse [x, y, radius, ...] parameter values into finite circle tuples."""
    values = [float(value) for value in raw_values]
    if len(values) % 3 != 0:
        raise ValueError('obstacles parameter length must be a multiple of 3')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('obstacles parameter must contain only finite values')
    if any(values[i + 2] <= 0.0 for i in range(0, len(values), 3)):
        raise ValueError('obstacle radii must be positive')
    return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)]
