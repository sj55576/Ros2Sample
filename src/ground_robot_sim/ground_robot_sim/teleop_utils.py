"""Pure key-mapping helpers for the keyboard teleop node (no ROS imports)."""

from typing import Optional, Tuple

Command = Tuple[float, float, float]
"""Unit direction multipliers as (linear_x, linear_z, angular_z)."""

MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 0.0, 1.0),
    'd': (0.0, 0.0, -1.0),
    'r': (0.0, 1.0, 0.0),
    'f': (0.0, -1.0, 0.0),
}

SCALE_BINDINGS = {'q': 1.1, 'z': 0.9}

STOP_KEYS = frozenset({' ', 'x'})

SCALE_MIN = 0.05
SCALE_MAX = 5.0


def command_for_key(key: str) -> Optional[Command]:
    """Return the unit movement command bound to key, or None if unbound."""
    return MOVE_BINDINGS.get(key.lower())


def scale_factor_for_key(key: str) -> Optional[float]:
    """Return the scale multiplier bound to key, or None if unbound."""
    return SCALE_BINDINGS.get(key.lower())


def is_stop_key(key: str) -> bool:
    """Return True if key requests an immediate stop."""
    return key.lower() in STOP_KEYS


def clamp_scale(value: float) -> float:
    """Clamp value into the [SCALE_MIN, SCALE_MAX] range."""
    return max(SCALE_MIN, min(SCALE_MAX, value))


def scaled_velocity(
    command: Command,
    linear_scale: float,
    vertical_scale: float,
    angular_scale: float,
) -> Tuple[float, float, float]:
    """Scale a unit command by the current per-axis scale factors."""
    return (
        command[0] * linear_scale,
        command[1] * vertical_scale,
        command[2] * angular_scale,
    )
