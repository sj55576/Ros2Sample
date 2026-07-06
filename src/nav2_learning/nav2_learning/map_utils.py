"""Utility functions for OccupancyGrid manipulation and path planning."""

from collections import deque
import heapq
import math
from typing import Dict, List, Optional, Tuple


def create_empty_grid(width: int, height: int, default_value: int = 0) -> List[int]:
    """Return a flat occupancy grid initialized to default_value."""
    return [default_value] * (width * height)


def world_to_grid(
    wx: float,
    wy: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[int, int]:
    """Convert world coordinates to grid cell indices."""
    gx = int((wx - origin_x) / resolution)
    gy = int((wy - origin_y) / resolution)
    return gx, gy


def grid_to_world(
    gx: int,
    gy: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[float, float]:
    """Convert grid cell indices to world coordinates (cell center)."""
    wx = origin_x + (gx + 0.5) * resolution
    wy = origin_y + (gy + 0.5) * resolution
    return wx, wy


def draw_filled_rectangle(
    grid: List[int],
    width: int,
    height: int,
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    value: int = 100,
) -> None:
    """Fill a rectangle region [x1,y1]-[x2,y2] with the given value (in-place)."""
    cx1 = max(0, min(x1, x2))
    cx2 = min(width - 1, max(x1, x2))
    cy1 = max(0, min(y1, y2))
    cy2 = min(height - 1, max(y1, y2))
    for gy in range(cy1, cy2 + 1):
        for gx in range(cx1, cx2 + 1):
            grid[gy * width + gx] = value


def draw_filled_circle(
    grid: List[int],
    width: int,
    height: int,
    cx: int,
    cy: int,
    radius: int,
    value: int = 100,
) -> None:
    """Fill a circular region centered at (cx,cy) with the given value (in-place)."""
    r2 = radius * radius
    for gy in range(max(0, cy - radius), min(height, cy + radius + 1)):
        for gx in range(max(0, cx - radius), min(width, cx + radius + 1)):
            dx = gx - cx
            dy = gy - cy
            if dx * dx + dy * dy <= r2:
                grid[gy * width + gx] = value


def draw_walls(grid: List[int], width: int, height: int, value: int = 100) -> None:
    """Draw walls around the grid border (1 cell thick)."""
    draw_filled_rectangle(grid, width, height, 0, 0, width - 1, 0, value)
    draw_filled_rectangle(grid, width, height, 0, height - 1, width - 1, height - 1, value)
    draw_filled_rectangle(grid, width, height, 0, 0, 0, height - 1, value)
    draw_filled_rectangle(grid, width, height, width - 1, 0, width - 1, height - 1, value)


def inflate_obstacles(
    grid: List[int],
    width: int,
    height: int,
    inflation_radius: int,
    max_cost: int = 99,
) -> List[int]:
    """Return a new grid with inflated obstacles using distance-based cost."""
    result = list(grid)
    # BFS from all occupied cells; track the minimum distance to an obstacle
    # for each free cell to assign a gradient cost.
    dist = [math.inf] * (width * height)
    queue: deque = deque()

    for idx in range(width * height):
        if grid[idx] > 0:
            dist[idx] = 0.0
            queue.append(idx)

    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    while queue:
        idx = queue.popleft()
        gy = idx // width
        gx = idx % width
        current_dist = dist[idx]

        for dx, dy in directions:
            nx, ny = gx + dx, gy + dy
            if not is_valid_cell(nx, ny, width, height):
                continue
            step = math.sqrt(dx * dx + dy * dy)
            new_dist = current_dist + step
            nidx = ny * width + nx
            if new_dist < dist[nidx] and new_dist < inflation_radius:
                dist[nidx] = new_dist
                queue.append(nidx)

    for idx in range(width * height):
        if grid[idx] > 0:
            result[idx] = 100
        elif dist[idx] < inflation_radius:
            factor = 1.0 - dist[idx] / inflation_radius
            result[idx] = max(result[idx], int(max_cost * factor))

    return result


def is_valid_cell(gx: int, gy: int, width: int, height: int) -> bool:
    """Check if grid coordinates are within bounds."""
    return 0 <= gx < width and 0 <= gy < height


def get_cell(grid: List[int], gx: int, gy: int, width: int) -> int:
    """Get the value at grid position (gx, gy). Row-major order: index = gy * width + gx."""
    return grid[gy * width + gx]


def a_star_search(
    grid: List[int],
    width: int,
    height: int,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    cost_threshold: int = 50,
    allow_diagonal: bool = True,
    resolution: float = 1.0,
) -> Optional[List[Tuple[int, int]]]:
    """Run A* from start to goal on a flat occupancy grid; return cell path or None."""
    def heuristic(gx: int, gy: int) -> float:
        dx = (gx - goal[0]) * resolution
        dy = (gy - goal[1]) * resolution
        return math.sqrt(dx * dx + dy * dy)

    def is_passable(gx: int, gy: int) -> bool:
        if not is_valid_cell(gx, gy, width, height):
            return False
        cost = grid[gy * width + gx]
        return cost != -1 and cost < cost_threshold

    cardinal = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    diagonal_dirs = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
    neighbors = cardinal + diagonal_dirs if allow_diagonal else cardinal

    open_heap: List[Tuple[float, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (heuristic(*start), start))

    g_scores: Dict[Tuple[int, int], float] = {start: 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        gx, gy = current
        for dx, dy in neighbors:
            nx, ny = gx + dx, gy + dy
            if not is_passable(nx, ny):
                continue
            step_cost = math.sqrt(dx * dx + dy * dy) * resolution
            tentative_g = g_scores[current] + step_cost
            neighbor = (nx, ny)
            if tentative_g < g_scores.get(neighbor, math.inf):
                g_scores[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + heuristic(nx, ny)
                heapq.heappush(open_heap, (f, neighbor))

    return None
