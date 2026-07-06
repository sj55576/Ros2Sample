"""Planar manipulator simulator publishing joint states, TF, and tool pose."""

import math
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from manipulator_sim.kinematics import forward_kinematics, step_towards
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def _yaw_to_quat_zw(yaw_rad: float) -> Tuple[float, float]:
    """Return quaternion (z, w) for yaw-only rotation where x and y are zero."""
    half = 0.5 * yaw_rad
    return math.sin(half), math.cos(half)


class ManipulatorSimulator(Node):
    """Simulate a simple 2-link planar arm with velocity-limited joint tracking."""

    def __init__(self) -> None:
        super().__init__('manipulator_simulator')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('link1_frame_id', 'link1')
        self.declare_parameter('link2_frame_id', 'link2')
        self.declare_parameter('tool_frame_id', 'tool0')
        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('link_lengths', [0.8, 0.6])
        self.declare_parameter('initial_joint_positions', [0.0, 0.0])
        self.declare_parameter('max_joint_speed', 1.2)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.link1_frame_id = str(self.get_parameter('link1_frame_id').value)
        self.link2_frame_id = str(self.get_parameter('link2_frame_id').value)
        self.tool_frame_id = str(self.get_parameter('tool_frame_id').value)
        self.joint_names = [str(name) for name in self.get_parameter('joint_names').value]
        if len(self.joint_names) != 2:
            raise ValueError('joint_names must contain exactly two names')

        raw_lengths = list(self.get_parameter('link_lengths').value)
        if len(raw_lengths) != 2:
            raise ValueError('link_lengths must contain exactly two values')
        self.link_lengths = (float(raw_lengths[0]), float(raw_lengths[1]))

        raw_initial = list(self.get_parameter('initial_joint_positions').value)
        if len(raw_initial) != 2:
            raise ValueError('initial_joint_positions must contain exactly two values')

        self.current_joint_positions = [float(raw_initial[0]), float(raw_initial[1])]
        self.target_joint_positions = list(self.current_joint_positions)

        self.max_joint_speed = max(0.0, float(self.get_parameter('max_joint_speed').value))
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._dt = 1.0 / publish_rate

        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._tool_pose_pub = self.create_publisher(PoseStamped, 'tool_pose', 10)
        self.create_subscription(JointState, 'joint_target', self._on_joint_target, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self._dt, self._tick)

    def _on_joint_target(self, msg: JointState) -> None:
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        for joint_index, joint_name in enumerate(self.joint_names):
            if joint_name in name_to_index:
                source_index = name_to_index[joint_name]
                if source_index < len(msg.position):
                    self.target_joint_positions[joint_index] = float(msg.position[source_index])

    def _tick(self) -> None:
        max_delta = self.max_joint_speed * self._dt
        for index in range(2):
            self.current_joint_positions[index] = step_towards(
                self.current_joint_positions[index],
                self.target_joint_positions[index],
                max_delta,
            )

        now = self.get_clock().now().to_msg()
        self._publish_joint_state(now)
        self._publish_tool_pose(now)
        self._publish_tf(now)

    def _publish_joint_state(self, stamp: Time) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = list(self.current_joint_positions)
        self._joint_pub.publish(msg)

    def _publish_tool_pose(self, stamp: Time) -> None:
        theta1, theta2 = self.current_joint_positions
        l1, l2 = self.link_lengths
        x, y = forward_kinematics(theta1, theta2, l1, l2)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        z, w = _yaw_to_quat_zw(theta1 + theta2)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self._tool_pose_pub.publish(pose)

    def _publish_tf(self, stamp: Time) -> None:
        theta1, theta2 = self.current_joint_positions
        l1, l2 = self.link_lengths

        base_to_link1 = TransformStamped()
        base_to_link1.header.stamp = stamp
        base_to_link1.header.frame_id = self.frame_id
        base_to_link1.child_frame_id = self.link1_frame_id
        base_to_link1.transform.translation.x = 0.0
        base_to_link1.transform.translation.y = 0.0
        base_to_link1.transform.translation.z = 0.0
        z1, w1 = _yaw_to_quat_zw(theta1)
        base_to_link1.transform.rotation.z = z1
        base_to_link1.transform.rotation.w = w1

        link1_to_link2 = TransformStamped()
        link1_to_link2.header.stamp = stamp
        link1_to_link2.header.frame_id = self.link1_frame_id
        link1_to_link2.child_frame_id = self.link2_frame_id
        link1_to_link2.transform.translation.x = l1
        link1_to_link2.transform.translation.y = 0.0
        link1_to_link2.transform.translation.z = 0.0
        z2, w2 = _yaw_to_quat_zw(theta2)
        link1_to_link2.transform.rotation.z = z2
        link1_to_link2.transform.rotation.w = w2

        link2_to_tool = TransformStamped()
        link2_to_tool.header.stamp = stamp
        link2_to_tool.header.frame_id = self.link2_frame_id
        link2_to_tool.child_frame_id = self.tool_frame_id
        link2_to_tool.transform.translation.x = l2
        link2_to_tool.transform.translation.y = 0.0
        link2_to_tool.transform.translation.z = 0.0
        link2_to_tool.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform([base_to_link1, link1_to_link2, link2_to_tool])


def main(args: Optional[List[str]] = None) -> None:
    """Run the manipulator simulator node."""
    rclpy.init(args=args)
    node = ManipulatorSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
