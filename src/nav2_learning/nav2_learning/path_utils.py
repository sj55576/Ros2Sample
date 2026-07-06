"""
Pure functions for path shortcutting, smoothing, and blockage detection.

These helpers operate on flat OccupancyGrid-style data and plain cell/world
coordinate tuples only; they do not depend on ROS message types so they can be
unit-tested without rclpy.
"""

from typing import List, Tuple

from nav2_learning.map_utils import get_cell, is_valid_cell
from nav2_learning.mapping_utils import bresenham_line


def has_line_of_sight(
    grid: List[int],
    width: int,
    height: int,
    a: Tuple[int, int],
    b: Tuple[int, int],
    cost_threshold: int,
) -> bool:
    """Return True if every cell on the straight line between a and b is passable."""
    for gx, gy in bresenham_line(a[0], a[1], b[0], b[1]):
        if not is_valid_cell(gx, gy, width, height):
            return False
        cost = get_cell(grid, gx, gy, width)
        if cost == -1 or cost >= cost_threshold:
            return False
    return True


def shortcut_path(
    grid: List[int],
    width: int,
    height: int,
    path: List[Tuple[int, int]],
    cost_threshold: int,
) -> List[Tuple[int, int]]:
    """Greedily remove redundant waypoints by skipping ahead to the farthest visible one."""
    if len(path) <= 2:
        return list(path)

    last_index = len(path) - 1
    result: List[Tuple[int, int]] = [path[0]]
    current_index = 0

    while current_index < last_index:
        next_index = current_index + 1
        for candidate_index in range(last_index, current_index, -1):
            if has_line_of_sight(
                grid, width, height, path[current_index], path[candidate_index], cost_threshold
            ):
                next_index = candidate_index
                break
        result.append(path[next_index])
        current_index = next_index

    return result


def smooth_path_moving_average(
    points: List[Tuple[float, float]],
    window: int,
) -> List[Tuple[float, float]]:
    """Smooth a world-coordinate point list with a moving average, keeping endpoints fixed."""
    if len(points) < 3:
        return list(points)

    window = max(1, window)
    if window % 2 == 0:
        window += 1
    half_window = window // 2

    last_index = len(points) - 1
    smoothed: List[Tuple[float, float]] = [points[0]]

    for i in range(1, last_index):
        lo = max(0, i - half_window)
        hi = min(last_index, i + half_window)
        neighborhood = points[lo:hi + 1]
        avg_x = sum(p[0] for p in neighborhood) / len(neighborhood)
        avg_y = sum(p[1] for p in neighborhood) / len(neighborhood)
        smoothed.append((avg_x, avg_y))

    smoothed.append(points[-1])
    return smoothed


def is_path_blocked(
    grid: List[int],
    width: int,
    height: int,
    path_cells: List[Tuple[int, int]],
    cost_threshold: int,
) -> bool:
    """Return True if any cell of path_cells is out of bounds, unknown, or above threshold."""
    for gx, gy in path_cells:
        if not is_valid_cell(gx, gy, width, height):
            return True
        cost = get_cell(grid, gx, gy, width)
        if cost == -1 or cost >= cost_threshold:
            return True
    return False
