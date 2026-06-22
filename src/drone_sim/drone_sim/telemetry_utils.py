"""Pure telemetry computation helpers (no ROS imports)."""

import json
import math


def compute_distance(
    x1: float, y1: float, z1: float,
    x2: float, y2: float, z2: float,
) -> float:
    """Euclidean distance between two 3D points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def compute_speed(vx: float, vy: float, vz: float) -> float:
    """Compute scalar speed from velocity components."""
    return math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)


def format_telemetry(
    total_distance_m: float,
    max_altitude_m: float,
    max_speed_ms: float,
    battery_pct: float,
    flight_time_sec: float,
    current_x: float,
    current_y: float,
    current_z: float,
) -> str:
    """Format telemetry data as a JSON string."""
    return json.dumps({
        'total_distance_m': total_distance_m,
        'max_altitude_m': max_altitude_m,
        'max_speed_ms': max_speed_ms,
        'battery_pct': battery_pct,
        'flight_time_sec': flight_time_sec,
        'current_x': current_x,
        'current_y': current_y,
        'current_z': current_z,
    })
