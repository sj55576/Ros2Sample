"""Pure helpers for converting ROS pose samples to OpenUSD time samples."""

import math
from typing import Tuple


def stamp_to_seconds(sec: int, nanosec: int) -> float:
    """Convert a ROS builtin_interfaces/Time pair to seconds."""
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('invalid ROS timestamp')
    return float(sec) + float(nanosec) * 1.0e-9


def relative_time_code(
    stamp_seconds: float,
    first_stamp_seconds: float,
    time_codes_per_second: float,
) -> float:
    """Convert an absolute timestamp to a non-negative USD time code."""
    if (
        time_codes_per_second <= 0.0
        or not math.isfinite(time_codes_per_second)
    ):
        raise ValueError('time_codes_per_second must be finite and positive')
    if (
        not math.isfinite(stamp_seconds)
        or not math.isfinite(first_stamp_seconds)
    ):
        raise ValueError('timestamps must be finite')
    elapsed = max(0.0, stamp_seconds - first_stamp_seconds)
    return elapsed * time_codes_per_second


def normalized_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float, float]:
    """Return a unit quaternion, using identity for a near-zero input."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion components must be finite')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(value / norm for value in values)
