"""Publish joint targets by converting planar tool targets through inverse kinematics."""

from math import fabs
from typing import List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time
from manipulator_sim.kinematics import inverse_kinematics, parse_targets_xy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TargetCommander(Node):
    """Cycle through x,y targets and publish matching joint commands."""

    def __init__(self) -> None:
        super().__init__('target_commander')
        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('link_lengths', [0.8, 0.6])
        self.declare_parameter('targets_xy', [1.0, 0.2, 0.8, 0.8, 0.4, 0.9, 1.1, -0.2])
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('hold_time_sec', 1.0)
        self.declare_parameter('tolerance_rad', 0.03)
        self.declare_parameter('loop', True)
        self.declare_parameter('elbow_up', False)

        self.joint_names = [str(name) for name in self.get_parameter('joint_names').value]
        if len(self.joint_names) != 2:
            raise ValueError('joint_names must contain exactly two names')

        raw_lengths = list(self.get_parameter('link_lengths').value)
        if len(raw_lengths) != 2:
            raise ValueError('link_lengths must contain exactly two values')
        l1 = float(raw_lengths[0])
        l2 = float(raw_lengths[1])

        targets_xy = parse_targets_xy(list(self.get_parameter('targets_xy').value))
        elbow_up = bool(self.get_parameter('elbow_up').value)
        self.joint_targets = [
            inverse_kinematics(x, y, l1, l2, elbow_up=elbow_up)
            for x, y in targets_xy
        ]

        self.current_joint_positions = [0.0, 0.0]
        self.current_index = 0
        self.arrival_time = None

        self._command_pub = self.create_publisher(JointState, 'joint_target', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)

        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(f'Loaded {len(self.joint_targets)} planar target(s)')

    def _on_joint_states(self, msg: JointState) -> None:
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        for joint_index, joint_name in enumerate(self.joint_names):
            source_index = name_to_index.get(joint_name)
            if source_index is not None and source_index < len(msg.position):
                self.current_joint_positions[joint_index] = float(msg.position[source_index])

    def _tick(self) -> None:
        now = self.get_clock().now()
        target_joint_positions = self.joint_targets[self.current_index]
        self._publish_joint_target(now.to_msg(), target_joint_positions)

        tolerance = float(self.get_parameter('tolerance_rad').value)
        hold_time = float(self.get_parameter('hold_time_sec').value)
        if self._is_reached(target_joint_positions, tolerance):
            if self.arrival_time is None:
                self.arrival_time = now
                self.get_logger().info(
                    f'Reached target {self.current_index}: '
                    f'({target_joint_positions[0]:.3f}, {target_joint_positions[1]:.3f})'
                )
            elif (now - self.arrival_time).nanoseconds * 1e-9 >= hold_time:
                self._advance_target()
        else:
            self.arrival_time = None

    def _is_reached(
        self,
        target_joint_positions: Tuple[float, float],
        tolerance: float,
    ) -> bool:
        return (
            fabs(target_joint_positions[0] - self.current_joint_positions[0]) <= tolerance
            and fabs(target_joint_positions[1] - self.current_joint_positions[1]) <= tolerance
        )

    def _advance_target(self) -> None:
        self.arrival_time = None
        if self.current_index + 1 < len(self.joint_targets):
            self.current_index += 1
        elif bool(self.get_parameter('loop').value):
            self.current_index = 0
        self.get_logger().info(
            f'Commanding target {self.current_index}: {self.joint_targets[self.current_index]}'
        )

    def _publish_joint_target(
        self,
        stamp: Time,
        target_joint_positions: Tuple[float, float],
    ) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [float(target_joint_positions[0]), float(target_joint_positions[1])]
        self._command_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """Run the target commander node."""
    rclpy.init(args=args)
    node = TargetCommander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
