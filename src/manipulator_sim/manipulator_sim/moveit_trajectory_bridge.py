"""Bridge a MoveIt2 JointTrajectory plan into manipulator_sim joint targets."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from manipulator_sim.trajectory_utils import sample_joint_trajectory


class MoveItTrajectoryBridge(Node):
    """Subscribe to a planned trajectory and replay it as JointState targets."""

    def __init__(self) -> None:
        """Initialise parameters, subscription, publisher, and replay timer."""
        super().__init__('moveit_trajectory_bridge')

        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('input_topic', 'planned_joint_trajectory')
        self.declare_parameter('output_topic', 'joint_target')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('hold_last_point', True)

        self._joint_names = [str(name) for name in self.get_parameter('joint_names').value]
        if not self._joint_names:
            raise ValueError('joint_names must contain at least one joint')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._loop_trajectory = bool(self.get_parameter('loop_trajectory').value)
        self._hold_last_point = bool(self.get_parameter('hold_last_point').value)

        self._trajectory: Optional[JointTrajectory] = None
        self._start_time_sec = 0.0
        self._completed = False

        self.create_subscription(
            JointTrajectory,
            input_topic,
            self._on_trajectory,
            10,
        )
        self._joint_pub = self.create_publisher(JointState, output_topic, 10)
        self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f'MoveItTrajectoryBridge listening on {input_topic!r}, '
            f'publishing {output_topic!r}'
        )

    def _now_seconds(self) -> float:
        now = self.get_clock().now().nanoseconds
        return float(now) * 1e-9

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warn('Ignoring empty planned trajectory')
            return

        self._trajectory = msg
        self._start_time_sec = self._now_seconds()
        self._completed = False
        self.get_logger().info(
            f'Received trajectory with {len(msg.points)} points for '
            f'{list(msg.joint_names)}'
        )

    def _tick(self) -> None:
        if self._trajectory is None:
            return
        if self._completed and not self._hold_last_point:
            return

        elapsed_sec = self._now_seconds() - self._start_time_sec
        try:
            positions, complete = sample_joint_trajectory(
                self._trajectory.joint_names,
                self._trajectory.points,
                self._joint_names,
                elapsed_sec,
                self._loop_trajectory,
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid trajectory: {exc}')
            self._trajectory = None
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = list(positions)
        self._joint_pub.publish(msg)

        self._completed = complete and not self._loop_trajectory


def main(args: Optional[List[str]] = None) -> None:
    """Run the MoveIt trajectory bridge node."""
    rclpy.init(args=args)
    node = MoveItTrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
