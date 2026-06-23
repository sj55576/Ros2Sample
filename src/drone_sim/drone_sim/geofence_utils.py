"""Pure geofence boundary helpers (no ROS imports)."""

from typing import Tuple

from drone_sim.math_utils import clamp


def check_boundary(
    x: float,
    y: float,
    z: float,
    bounds_min: Tuple[float, float, float],
    bounds_max: Tuple[float, float, float],
    margin: float,
) -> Tuple[str, Tuple[float, float, float]]:
    """Return ('safe'|'warning'|'breach', clamped_position)."""
    safe_min = (bounds_min[0] + margin, bounds_min[1] + margin, bounds_min[2] + margin)
    safe_max = (bounds_max[0] - margin, bounds_max[1] - margin, bounds_max[2] - margin)

    clamped = (
        clamp(x, safe_min[0], safe_max[0]),
        clamp(y, safe_min[1], safe_max[1]),
        clamp(z, safe_min[2], safe_max[2]),
    )

    breached = (
        x < bounds_min[0] or x >= bounds_max[0]
        or y < bounds_min[1] or y >= bounds_max[1]
        or z < bounds_min[2] or z >= bounds_max[2]
    )
    if breached:
        return ('breach', clamped)

    in_warning = (
        x < safe_min[0] or x >= safe_max[0]
        or y < safe_min[1] or y >= safe_max[1]
        or z < safe_min[2] or z >= safe_max[2]
    )
    if in_warning:
        return ('warning', clamped)

    return ('safe', clamped)
