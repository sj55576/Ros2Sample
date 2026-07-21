"""Record nav_msgs/Odometry poses as time-sampled OpenUSD transforms."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from openusd_bridge.sampling import (
    normalized_quaternion,
    relative_time_code,
    stamp_to_seconds,
)
from openusd_bridge.usd_stage import RobotPoseStage


class OdomToUsd(Node):
    """Subscribe to odometry and record the robot pose in an OpenUSD stage."""

    def __init__(self) -> None:
        """Create the stage writer and odometry subscription."""
        super().__init__('odom_to_usd')
        self.declare_parameter('input_topic', 'odom')
        self.declare_parameter(
            'output_path', '/tmp/ros2_openusd/robot_motion.usda',
        )
        self.declare_parameter('robot_prim_path', '/World/Robot')
        self.declare_parameter('time_codes_per_second', 30.0)
        self.declare_parameter('save_every_n_samples', 30)

        input_topic = str(self.get_parameter('input_topic').value)
        output_path = str(self.get_parameter('output_path').value)
        robot_prim_path = str(self.get_parameter('robot_prim_path').value)
        self._time_codes_per_second = float(
            self.get_parameter('time_codes_per_second').value,
        )
        self._save_every_n_samples = max(
            1, int(self.get_parameter('save_every_n_samples').value),
        )
        self._stage = RobotPoseStage(
            output_path,
            robot_prim_path,
            self._time_codes_per_second,
        )
        self._first_stamp_seconds = None
        self._sample_count = 0
        self.create_subscription(Odometry, input_topic, self._on_odom, 10)
        self.get_logger().info(
            f'Recording {input_topic} to {self._stage.output_path}',
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Write one odometry pose as an OpenUSD time sample."""
        stamp_seconds = stamp_to_seconds(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )
        if self._first_stamp_seconds is None:
            self._first_stamp_seconds = stamp_seconds
        time_code = relative_time_code(
            stamp_seconds,
            self._first_stamp_seconds,
            self._time_codes_per_second,
        )
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = normalized_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._stage.add_pose(
            time_code,
            (position.x, position.y, position.z),
            quaternion,
        )
        self._sample_count += 1
        if self._sample_count % self._save_every_n_samples == 0:
            self._stage.save()

    def destroy_node(self) -> bool:
        """Save the stage before releasing the ROS node."""
        self._stage.save()
        self.get_logger().info(
            f'Saved {self._sample_count} samples to {self._stage.output_path}',
        )
        return super().destroy_node()


def main(args=None) -> None:
    """Run the odometry-to-OpenUSD recorder."""
    rclpy.init(args=args)
    node = None
    try:
        node = OdomToUsd()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
