"""Helpers for sampling joint trajectories without depending on ROS runtime."""

import math
from typing import Any, Sequence, Tuple


def duration_to_seconds(duration: Any) -> float:
    """Convert a ROS-like Duration object into seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def point_time_seconds(point: Any) -> float:
    """Return a trajectory point's ``time_from_start`` in seconds."""
    return duration_to_seconds(point.time_from_start)


def _positions_for_names(
    trajectory_joint_names: Sequence[str],
    point: Any,
    output_joint_names: Sequence[str],
) -> Tuple[float, ...]:
    name_to_index = {name: index for index, name in enumerate(trajectory_joint_names)}
    positions = tuple(float(value) for value in point.positions)

    selected = []
    for joint_name in output_joint_names:
        if joint_name not in name_to_index:
            raise ValueError(f'trajectory is missing joint {joint_name!r}')
        source_index = name_to_index[joint_name]
        if source_index >= len(positions):
            raise ValueError(f'trajectory point has no position for {joint_name!r}')
        selected.append(positions[source_index])

    return tuple(selected)


def sample_joint_trajectory(
    trajectory_joint_names: Sequence[str],
    points: Sequence[Any],
    output_joint_names: Sequence[str],
    elapsed_sec: float,
    loop: bool = False,
) -> Tuple[Tuple[float, ...], bool]:
    """Sample trajectory positions at elapsed seconds.

    Returns ``(positions, complete)``. Positions are linearly interpolated between
    neighboring trajectory points and reordered to match ``output_joint_names``.
    """
    if not points:
        raise ValueError('trajectory must contain at least one point')
    if not output_joint_names:
        raise ValueError('output_joint_names must contain at least one joint')
    if not math.isfinite(elapsed_sec):
        raise ValueError('elapsed_sec must be finite')

    times = [point_time_seconds(point) for point in points]
    if any(not math.isfinite(time_value) for time_value in times):
        raise ValueError('trajectory point times must be finite')
    if any(next_time < time_value for time_value, next_time in zip(times, times[1:])):
        raise ValueError('trajectory point times must be monotonic')

    trajectory_duration = times[-1]
    sample_time = max(0.0, elapsed_sec)
    if loop and trajectory_duration > 0.0:
        sample_time = sample_time % trajectory_duration

    if sample_time <= times[0]:
        return (
            _positions_for_names(trajectory_joint_names, points[0], output_joint_names),
            False,
        )

    for index in range(1, len(points)):
        prev_time = times[index - 1]
        next_time = times[index]
        if sample_time <= next_time:
            prev_positions = _positions_for_names(
                trajectory_joint_names,
                points[index - 1],
                output_joint_names,
            )
            next_positions = _positions_for_names(
                trajectory_joint_names,
                points[index],
                output_joint_names,
            )
            span = next_time - prev_time
            if span <= 0.0:
                return next_positions, False

            ratio = (sample_time - prev_time) / span
            interpolated = tuple(
                prev + (next_value - prev) * ratio
                for prev, next_value in zip(prev_positions, next_positions)
            )
            return interpolated, False

    return (
        _positions_for_names(trajectory_joint_names, points[-1], output_joint_names),
        True,
    )
