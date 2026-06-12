"""Pure waypoint-parsing helpers for the drone simulation (no ROS imports)."""

from typing import List, Sequence, Tuple


Point3 = Tuple[float, float, float]


def parse_waypoints(raw_waypoints: Sequence[float]) -> List[Point3]:
    """Parse a flat [x, y, z, ...] sequence into a list of Point3 triples."""
    if len(raw_waypoints) < 3 or len(raw_waypoints) % 3 != 0:
        raise ValueError('waypoints parameter must contain x, y, z triples')
    return [
        (
            float(raw_waypoints[index]),
            float(raw_waypoints[index + 1]),
            float(raw_waypoints[index + 2]),
        )
        for index in range(0, len(raw_waypoints), 3)
    ]
