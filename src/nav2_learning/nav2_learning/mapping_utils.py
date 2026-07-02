"""Pure functions for log-odds occupancy grid mapping from laser scans."""

import math
from typing import List, Sequence, Tuple

from nav2_learning.map_utils import is_valid_cell, world_to_grid


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Return all integer grid cells on the line from (x0, y0) to (x1, y1), inclusive."""
    cells: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0

    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy

    return cells


def prob_to_log_odds(probability: float) -> float:
    """Convert a probability in (0, 1) to a log-odds value: log(p / (1 - p))."""
    return math.log(probability / (1.0 - probability))


def log_odds_to_prob(log_odds: float) -> float:
    """Convert a log-odds value to a probability, numerically stable for large |log_odds|."""
    if log_odds >= 0:
        return 1.0 / (1.0 + math.exp(-log_odds))
    ex = math.exp(log_odds)
    return ex / (1.0 + ex)


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value into [low, high]."""
    return max(low, min(high, value))


def integrate_scan(
    log_odds: List[float],
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    hit_log_odds: float = 0.85,
    miss_log_odds: float = -0.4,
    log_odds_min: float = -4.0,
    log_odds_max: float = 4.0,
) -> None:
    """Integrate one laser scan into the log-odds grid in place (inverse sensor model)."""
    sensor_gx, sensor_gy = world_to_grid(sensor_x, sensor_y, origin_x, origin_y, resolution)
    if not is_valid_cell(sensor_gx, sensor_gy, width, height):
        return

    for i, scan_range in enumerate(ranges):
        if math.isnan(scan_range) or scan_range < range_min or scan_range <= 0:
            continue

        angle = sensor_yaw + angle_min + i * angle_increment
        is_hit = scan_range < range_max
        used_range = scan_range if is_hit else range_max

        end_x = sensor_x + used_range * math.cos(angle)
        end_y = sensor_y + used_range * math.sin(angle)
        end_gx, end_gy = world_to_grid(end_x, end_y, origin_x, origin_y, resolution)

        cells = bresenham_line(sensor_gx, sensor_gy, end_gx, end_gy)
        last_index = len(cells) - 1
        for idx, (gx, gy) in enumerate(cells):
            if not is_valid_cell(gx, gy, width, height):
                continue
            delta = hit_log_odds if (idx == last_index and is_hit) else miss_log_odds
            cell_index = gy * width + gx
            log_odds[cell_index] = clamp(
                log_odds[cell_index] + delta, log_odds_min, log_odds_max
            )


def log_odds_to_occupancy(
    log_odds: Sequence[float],
    occupied_threshold: float = 0.65,
    free_threshold: float = 0.35,
) -> List[int]:
    """Convert log-odds values to OccupancyGrid data (100 occupied / 0 free / -1 unknown)."""
    occupancy: List[int] = []
    for value in log_odds:
        probability = log_odds_to_prob(value)
        if probability > occupied_threshold:
            occupancy.append(100)
        elif probability < free_threshold:
            occupancy.append(0)
        else:
            occupancy.append(-1)
    return occupancy
