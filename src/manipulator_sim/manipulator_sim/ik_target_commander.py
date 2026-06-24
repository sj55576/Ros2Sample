"""Node that converts Cartesian pose targets to joint commands via inverse kinematics."""

from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from manipulator_sim.inverse_kinematics import nearest_reachable, solve_ik


class IkTargetCommander(Node):
    """Subscribe to a Cartesian target and publish IK-solved joint commands."""

    def __init__(self) -> None:
        """Initialise parameters, pub/sub, and timer."""
        super().__init__('ik_target_commander')

        self.declare_parameter('link_lengths', [0.8, 0.6])
        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('elbow_up', True)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('clamp_to_workspace', True)

        raw_lengths = list(self.get_parameter('link_lengths').value)
        if len(raw_lengths) != 2:
            raise ValueError('link_lengths must contain exactly two values')
        self._l1 = float(raw_lengths[0])
        self._l2 = float(raw_lengths[1])
        self._joint_names: List[str] = [
            str(n) for n in self.get_parameter('joint_names').value
        ]
        self._elbow_up = bool(self.get_parameter('elbow_up').value)
        self._clamp = bool(self.get_parameter('clamp_to_workspace').value)

        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._last_solution: Optional[tuple] = None

        self.create_subscription(
            PoseStamped, 'target_pose', self._on_target, 10,
        )
        self._joint_pub = self.create_publisher(JointState, 'joint_target', 10)

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'IkTargetCommander started: l1={self._l1}, l2={self._l2}, '
            f'elbow_up={self._elbow_up}'
        )

    def _on_target(self, msg: PoseStamped) -> None:
        """Store the latest Cartesian target from an incoming PoseStamped."""
        self._target_x = msg.pose.position.x
        self._target_y = msg.pose.position.y

    def _tick(self) -> None:
        """Solve IK and publish joint target on each timer tick."""
        if self._target_x is None or self._target_y is None:
            return

        x, y = self._target_x, self._target_y
        solution = solve_ik(x, y, self._l1, self._l2, self._elbow_up)

        if solution is None and self._clamp:
            cx, cy = nearest_reachable(x, y, self._l1, self._l2)
            solution = solve_ik(cx, cy, self._l1, self._l2, self._elbow_up)
            if solution is not None:
                self.get_logger().debug(
                    f'Target ({x:.3f}, {y:.3f}) unreachable; '
                    f'clamped to ({cx:.3f}, {cy:.3f})'
                )

        if solution is None:
            if self._last_solution is not None:
                self.get_logger().warn(
                    f'No IK solution for ({x:.3f}, {y:.3f}); holding last pose'
                )
            return

        self._last_solution = solution
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [solution[0], solution[1]]
        self._joint_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """Run the IK target commander node."""
    rclpy.init(args=args)
    node = IkTargetCommander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
