"""Pure wind computation helpers (no ROS imports)."""

import math
from typing import Tuple

_TURB_PRIME_X = 127.1
_TURB_PRIME_Y = 311.7
_TURB_PRIME_Z = 74.3


def compute_wind(
    base: Tuple[float, float, float],
    gust_amplitude: float,
    gust_period_sec: float,
    turbulence_intensity: float,
    elapsed_sec: float,
) -> Tuple[float, float, float]:
    """Return (wx, wy, wz) wind vector for the given elapsed time and parameters."""
    if gust_period_sec > 0.0:
        angle = 2.0 * math.pi * elapsed_sec / gust_period_sec
        gust_x = gust_amplitude * math.sin(angle)
        gust_y = gust_amplitude * math.sin(angle + math.pi / 2.0)
    else:
        gust_x = 0.0
        gust_y = 0.0

    turb_x = turbulence_intensity * math.sin(elapsed_sec * _TURB_PRIME_X)
    turb_y = turbulence_intensity * math.sin(elapsed_sec * _TURB_PRIME_Y)
    turb_z = turbulence_intensity * math.sin(elapsed_sec * _TURB_PRIME_Z)

    return (
        base[0] + gust_x + turb_x,
        base[1] + gust_y + turb_y,
        base[2] + turb_z,
    )
