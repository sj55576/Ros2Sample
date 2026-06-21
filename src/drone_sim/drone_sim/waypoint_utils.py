"""Pure waypoint-parsing helpers for the drone simulation (no ROS imports)."""

import math
from typing import List, Sequence, Tuple


Point3 = Tuple[float, float, float]


def parse_waypoints(raw_waypoints: Sequence[float]) -> List[Point3]:
    """Parse a flat [x, y, z, ...] sequence into finite Point3 triples."""
    if len(raw_waypoints) < 3 or len(raw_waypoints) % 3 != 0:
        raise ValueError('waypoints parameter must contain x, y, z triples')

    values = [float(value) for value in raw_waypoints]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('waypoints parameter must contain only finite values')

    return [
        (values[index], values[index + 1], values[index + 2])
        for index in range(0, len(values), 3)
    ]
