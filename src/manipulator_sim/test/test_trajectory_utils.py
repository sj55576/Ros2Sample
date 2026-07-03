"""Tests for trajectory sampling helpers."""

from dataclasses import dataclass
from typing import Tuple

import pytest

from manipulator_sim.trajectory_utils import sample_joint_trajectory


@dataclass
class Duration:
    """Small stand-in for builtin_interfaces/Duration."""

    sec: int
    nanosec: int = 0


@dataclass
class Point:
    """Small stand-in for trajectory_msgs/JointTrajectoryPoint."""

    positions: Tuple[float, ...]
    time_from_start: Duration


def test_sample_joint_trajectory_interpolates_between_points() -> None:
    """Trajectory sampling should linearly interpolate joint positions."""
    points = [
        Point((0.0, 1.0), Duration(0)),
        Point((1.0, 3.0), Duration(2)),
    ]

    positions, complete = sample_joint_trajectory(
        ['joint1', 'joint2'],
        points,
        ['joint1', 'joint2'],
        1.0,
    )

    assert positions == pytest.approx((0.5, 2.0))
    assert complete is False


def test_sample_joint_trajectory_reorders_joint_names() -> None:
    """Output positions should match requested joint order."""
    points = [
        Point((0.0, 1.0), Duration(0)),
        Point((1.0, 3.0), Duration(2)),
    ]

    positions, _ = sample_joint_trajectory(
        ['joint1', 'joint2'],
        points,
        ['joint2', 'joint1'],
        2.0,
    )

    assert positions == pytest.approx((3.0, 1.0))


def test_sample_joint_trajectory_marks_completion_after_final_point() -> None:
    """Sampling past the last point should hold the final point and complete."""
    points = [
        Point((0.0, 0.0), Duration(0)),
        Point((1.0, -1.0), Duration(1)),
    ]

    positions, complete = sample_joint_trajectory(
        ['joint1', 'joint2'],
        points,
        ['joint1', 'joint2'],
        5.0,
    )

    assert positions == pytest.approx((1.0, -1.0))
    assert complete is True


def test_sample_joint_trajectory_loops_when_enabled() -> None:
    """Looping should wrap elapsed time into the trajectory duration."""
    points = [
        Point((0.0,), Duration(0)),
        Point((2.0,), Duration(2)),
    ]

    positions, complete = sample_joint_trajectory(
        ['joint1'],
        points,
        ['joint1'],
        3.0,
        loop=True,
    )

    assert positions == pytest.approx((1.0,))
    assert complete is False


def test_sample_joint_trajectory_rejects_missing_joint() -> None:
    """Sampling should fail when the requested joint is absent."""
    points = [Point((0.0,), Duration(0))]

    with pytest.raises(ValueError, match='missing joint'):
        sample_joint_trajectory(['joint1'], points, ['joint2'], 0.0)
