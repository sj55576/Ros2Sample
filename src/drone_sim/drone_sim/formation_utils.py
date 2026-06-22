"""Pure formation computation helpers (no ROS imports)."""

from typing import Tuple


def compute_formation_target(
    leader_x: float, leader_y: float, leader_z: float,
    offset_x: float, offset_y: float, offset_z: float,
) -> Tuple[float, float, float]:
    """Compute the target position for a follower given leader position and offset."""
    return (leader_x + offset_x, leader_y + offset_y, leader_z + offset_z)


def smooth_position(
    current: Tuple[float, float, float],
    target: Tuple[float, float, float],
    gain: float,
) -> Tuple[float, float, float]:
    """Apply exponential smoothing between current and target positions."""
    return (
        gain * target[0] + (1.0 - gain) * current[0],
        gain * target[1] + (1.0 - gain) * current[1],
        gain * target[2] + (1.0 - gain) * current[2],
    )
