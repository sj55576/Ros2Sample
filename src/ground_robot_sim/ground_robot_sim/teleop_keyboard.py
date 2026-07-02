"""Keyboard teleop node shared by ground robots and drones (issue #48).

Reads single keypresses from stdin and publishes ``geometry_msgs/Twist``
commands on ``cmd_vel``:

- ``w``/``s`` drive forward/backward, ``a``/``d`` turn left/right.
- ``r``/``f`` climb/descend (``linear.z``), useful for drones.
- ``q``/``z`` scale the speed up/down.
- space or ``x`` stops immediately.
- The robot auto-stops if no key is pressed within ``key_timeout_sec``.

To drive a specific robot in a multi-robot or drone demo, remap
``cmd_vel`` on the command line, for example::

    ros2 run ground_robot_sim teleop_keyboard --ros-args -r cmd_vel:=/drone_1/cmd_vel
"""

import select
import sys
import termios
import time
import tty
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from ground_robot_sim import teleop_utils
from rclpy.node import Node

USAGE = """\
Keyboard teleop controls
-------------------------
  w / s : drive forward / backward
  a / d : turn left / right
  r / f : climb / descend (linear.z, useful for drones)
  q / z : increase / decrease speed scale
  space or x : stop immediately
Ctrl-C to exit. The robot auto-stops if no key is pressed for a while.
"""


class TeleopKeyboard(Node):
    """Publish Twist commands from keyboard input, for robots or drones."""

    def __init__(self) -> None:
        super().__init__('teleop_keyboard')
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('vertical_scale', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('key_timeout_sec', 0.5)

        self.linear_scale = teleop_utils.clamp_scale(
            float(self.get_parameter('linear_scale').value)
        )
        self.angular_scale = teleop_utils.clamp_scale(
            float(self.get_parameter('angular_scale').value)
        )
        self.vertical_scale = teleop_utils.clamp_scale(
            float(self.get_parameter('vertical_scale').value)
        )
        self.key_timeout_sec = float(self.get_parameter('key_timeout_sec').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self._command: teleop_utils.Command = (0.0, 0.0, 0.0)
        self._last_key_time = time.monotonic()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        if not sys.stdin.isatty():
            self.get_logger().warn(
                'stdin is not a TTY; keyboard input may not be readable interactively.'
            )

        self.get_logger().info(USAGE)

    def _on_timer(self) -> None:
        """Drain pending keypresses, apply auto-stop, and publish a Twist."""
        for key in self._read_pending_keys():
            self._handle_key(key)

        now = time.monotonic()
        if self._command != (0.0, 0.0, 0.0) and (now - self._last_key_time) > self.key_timeout_sec:
            self._command = (0.0, 0.0, 0.0)
            self.get_logger().info('Auto-stop: no key received within timeout')

        self._publish_command()

    def _read_pending_keys(self) -> List[str]:
        """Non-blockingly read all characters currently waiting on stdin."""
        keys = []
        while select.select([sys.stdin], [], [], 0.0)[0]:
            char = sys.stdin.read(1)
            if not char:
                break
            keys.append(char)
        return keys

    def _handle_key(self, key: str) -> None:
        """Update scale/command state in response to a single keypress."""
        if teleop_utils.is_stop_key(key):
            self._command = (0.0, 0.0, 0.0)
            self.get_logger().info('stop')
            self._publish_command()
            return

        scale_factor = teleop_utils.scale_factor_for_key(key)
        if scale_factor is not None:
            self.linear_scale = teleop_utils.clamp_scale(self.linear_scale * scale_factor)
            self.vertical_scale = teleop_utils.clamp_scale(self.vertical_scale * scale_factor)
            self.angular_scale = teleop_utils.clamp_scale(self.angular_scale * scale_factor)
            self.get_logger().info(
                f'scale -> linear={self.linear_scale:.3f} '
                f'vertical={self.vertical_scale:.3f} angular={self.angular_scale:.3f}'
            )
            return

        command = teleop_utils.command_for_key(key)
        if command is not None:
            self._command = command
            self._last_key_time = time.monotonic()

    def _publish_command(self) -> None:
        """Publish the current command scaled by the active per-axis scales."""
        linear_x, linear_z, angular_z = teleop_utils.scaled_velocity(
            self._command, self.linear_scale, self.vertical_scale, self.angular_scale,
        )
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.z = linear_z
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)


def _enter_cbreak_mode() -> Optional[List]:
    """Put stdin into cbreak mode and return the saved attrs, if a TTY."""
    if not sys.stdin.isatty():
        return None
    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return old_attrs


def _restore_terminal(old_attrs: Optional[List]) -> None:
    """Restore previously saved stdin terminal attributes, if any."""
    if old_attrs is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopKeyboard()
    old_attrs = _enter_cbreak_mode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _restore_terminal(old_attrs)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
