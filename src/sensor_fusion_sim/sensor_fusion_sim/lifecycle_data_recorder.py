"""Lifecycle-managed data recorder demonstrating managed node transitions."""

import json
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    TransitionCallbackReturn,
)
from rclpy.lifecycle.node import LifecyclePublisher
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class LifecycleDataRecorder(LifecycleNode):
    """Record fused odometry data through lifecycle state transitions."""

    def __init__(self) -> None:
        super().__init__('lifecycle_data_recorder')

        self.declare_parameter('max_buffer_size', 500)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('input_topic', 'fused_odom')

        self._max_buffer_size = 0
        self._publish_rate_hz = 0.0
        self._input_topic = ''
        self._buffer: List[dict] = []
        self._sub = None
        self._timer = None
        self._status_pub: Optional[LifecyclePublisher] = None
        self._summary_pub: Optional[LifecyclePublisher] = None
        self._total_recorded = 0
        self._is_recording = False

        self.get_logger().info(
            'LifecycleDataRecorder created (unconfigured)'
        )

    def on_configure(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """Validate parameters and create publishers."""
        self._max_buffer_size = int(
            self.get_parameter('max_buffer_size').value
        )
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        self._input_topic = str(
            self.get_parameter('input_topic').value
        )

        if self._max_buffer_size <= 0:
            self.get_logger().error(
                'max_buffer_size must be positive'
            )
            return TransitionCallbackReturn.FAILURE

        self._buffer = []
        self._total_recorded = 0

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._status_pub = self.create_lifecycle_publisher(
            String, 'recording_status', reliable_qos,
        )
        self._summary_pub = self.create_lifecycle_publisher(
            String, 'recording_summary', reliable_qos,
        )

        self.get_logger().info(
            f'Configured: buffer_size={self._max_buffer_size}, '
            f'topic={self._input_topic}'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """Start recording by creating subscription and status timer."""
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            Odometry, self._input_topic,
            self._on_odom, reliable_qos,
        )

        period = 1.0 / max(self._publish_rate_hz, 0.1)
        self._timer = self.create_timer(
            period, self._publish_status,
        )

        self._is_recording = True
        self.get_logger().info('Activated: recording started')
        return super().on_activate(state)

    def on_deactivate(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """Pause recording and publish summary of buffered data."""
        self._is_recording = False

        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None

        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        self._publish_summary()
        self.get_logger().info(
            f'Deactivated: {len(self._buffer)} samples buffered'
        )
        return super().on_deactivate(state)

    def on_cleanup(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """Release resources and clear buffer."""
        self._buffer = []
        self._total_recorded = 0
        self._is_recording = False

        if self._status_pub is not None:
            self.destroy_publisher(self._status_pub)
            self._status_pub = None
        if self._summary_pub is not None:
            self.destroy_publisher(self._summary_pub)
            self._summary_pub = None

        self.get_logger().info('Cleaned up: buffer cleared')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """Handle shutdown from any state."""
        self._is_recording = False
        if self._buffer:
            self.get_logger().info(
                f'Shutdown: discarding {len(self._buffer)} samples'
            )
        self._buffer = []
        return TransitionCallbackReturn.SUCCESS

    def _on_odom(self, msg: Odometry) -> None:
        if not self._is_recording:
            return

        record = {
            'stamp': msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
        }

        if len(self._buffer) >= self._max_buffer_size:
            self._buffer.pop(0)
        self._buffer.append(record)
        self._total_recorded += 1

    def _publish_status(self) -> None:
        if self._status_pub is None:
            return
        if not self._status_pub.is_activated:
            return

        msg = String()
        msg.data = (
            f'recording={self._is_recording} '
            f'buffered={len(self._buffer)} '
            f'total={self._total_recorded} '
            f'max={self._max_buffer_size}'
        )
        self._status_pub.publish(msg)

    def _publish_summary(self) -> None:
        if self._summary_pub is None:
            return
        if not self._summary_pub.is_activated:
            return
        if not self._buffer:
            return

        xs = [r['x'] for r in self._buffer]
        ys = [r['y'] for r in self._buffer]

        summary = {
            'total_recorded': self._total_recorded,
            'buffered': len(self._buffer),
            'x_range': [min(xs), max(xs)],
            'y_range': [min(ys), max(ys)],
        }
        msg = String()
        msg.data = json.dumps(summary)
        self._summary_pub.publish(msg)
        self.get_logger().info(f'Summary: {msg.data}')


def main(args=None) -> None:
    """Entry point for the lifecycle_data_recorder executable."""
    rclpy.init(args=args)
    node = LifecycleDataRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
