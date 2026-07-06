"""Complementary filter node with callback groups and dynamic parameters."""

import math
import threading
from typing import Optional

from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ComplementaryFilterNode(Node):
    """Fuse GPS, IMU, and wheel odometry into a single pose estimate."""

    def __init__(self) -> None:
        super().__init__('complementary_filter')

        self.declare_parameter('gps_alpha', 0.15)
        self.declare_parameter('odom_alpha', 0.30)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link_fused')
        self.declare_parameter('imu_yaw_weight', 0.05)

        self._gps_alpha = float(
            self.get_parameter('gps_alpha').value
        )
        self._odom_alpha = float(
            self.get_parameter('odom_alpha').value
        )
        publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        self._frame_id = str(
            self.get_parameter('frame_id').value
        )
        self._child_frame_id = str(
            self.get_parameter('child_frame_id').value
        )
        self._imu_yaw_weight = float(
            self.get_parameter('imu_yaw_weight').value
        )

        self._lock = threading.Lock()
        self._fused_x = 0.0
        self._fused_y = 0.0
        self._fused_z = 0.0
        self._fused_yaw = 0.0
        self._fused_vx = 0.0
        self._fused_vy = 0.0
        self._last_gps: Optional[PointStamped] = None
        self._last_imu: Optional[Imu] = None
        self._last_odom: Optional[Odometry] = None
        self._gps_count = 0
        self._imu_count = 0
        self._odom_count = 0

        sensor_cb_group = ReentrantCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PointStamped, 'gps', self._on_gps, reliable_qos,
            callback_group=sensor_cb_group,
        )
        self.create_subscription(
            Imu, 'imu', self._on_imu, best_effort_qos,
            callback_group=sensor_cb_group,
        )
        self.create_subscription(
            Odometry, 'wheel_odom', self._on_odom, reliable_qos,
            callback_group=sensor_cb_group,
        )

        self._fused_pub = self.create_publisher(
            Odometry, 'fused_odom', reliable_qos,
        )
        self._diag_pub = self.create_publisher(
            String, 'filter_diagnostics', 10,
        )

        period = 1.0 / max(publish_rate, 0.1)
        self.create_timer(
            period, self._publish_fused,
            callback_group=timer_cb_group,
        )

        self.add_on_set_parameters_callback(
            self._on_param_change
        )

        self.get_logger().info(
            f'ComplementaryFilter started: '
            f'gps_alpha={self._gps_alpha:.2f}, '
            f'odom_alpha={self._odom_alpha:.2f}'
        )

    def _on_param_change(
        self, params: list,
    ) -> SetParametersResult:
        """Validate and apply dynamic parameter changes."""
        for param in params:
            if param.name == 'gps_alpha':
                val = float(param.value)
                if not 0.0 <= val <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='gps_alpha must be in [0, 1]',
                    )
            elif param.name == 'odom_alpha':
                val = float(param.value)
                if not 0.0 <= val <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='odom_alpha must be in [0, 1]',
                    )
            elif param.name == 'imu_yaw_weight':
                val = float(param.value)
                if not 0.0 <= val <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='imu_yaw_weight must be in [0, 1]',
                    )

        for param in params:
            if param.name == 'gps_alpha':
                self._gps_alpha = float(param.value)
                self.get_logger().info(
                    f'gps_alpha updated to {self._gps_alpha:.3f}'
                )
            elif param.name == 'odom_alpha':
                self._odom_alpha = float(param.value)
                self.get_logger().info(
                    f'odom_alpha updated to '
                    f'{self._odom_alpha:.3f}'
                )
            elif param.name == 'imu_yaw_weight':
                self._imu_yaw_weight = float(param.value)
                self.get_logger().info(
                    f'imu_yaw_weight updated to '
                    f'{self._imu_yaw_weight:.3f}'
                )

        return SetParametersResult(successful=True)

    def _on_gps(self, msg: PointStamped) -> None:
        with self._lock:
            self._last_gps = msg
            self._gps_count += 1
            self._fused_x = _blend(
                self._fused_x, msg.point.x, self._gps_alpha,
            )
            self._fused_y = _blend(
                self._fused_y, msg.point.y, self._gps_alpha,
            )
            self._fused_z = _blend(
                self._fused_z, msg.point.z, self._gps_alpha,
            )

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self._last_imu = msg
            self._imu_count += 1
            imu_yaw = _yaw_from_quaternion(msg.orientation)
            diff = _normalize_angle(imu_yaw - self._fused_yaw)
            self._fused_yaw = _normalize_angle(
                self._fused_yaw + self._imu_yaw_weight * diff
            )

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._last_odom = msg
            self._odom_count += 1
            pos = msg.pose.pose.position
            self._fused_x = _blend(
                self._fused_x, pos.x, self._odom_alpha,
            )
            self._fused_y = _blend(
                self._fused_y, pos.y, self._odom_alpha,
            )
            self._fused_z = _blend(
                self._fused_z, pos.z, self._odom_alpha,
            )
            self._fused_vx = msg.twist.twist.linear.x
            self._fused_vy = msg.twist.twist.linear.y

    def _publish_fused(self) -> None:
        with self._lock:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self._frame_id
            odom.child_frame_id = self._child_frame_id
            odom.pose.pose.position.x = self._fused_x
            odom.pose.pose.position.y = self._fused_y
            odom.pose.pose.position.z = self._fused_z
            odom.pose.pose.orientation = _yaw_to_quaternion(
                self._fused_yaw
            )
            odom.twist.twist.linear.x = self._fused_vx
            odom.twist.twist.linear.y = self._fused_vy
            self._fused_pub.publish(odom)

            diag = String()
            diag.data = (
                f'gps={self._gps_count} '
                f'imu={self._imu_count} '
                f'odom={self._odom_count} '
                f'pos=({self._fused_x:.3f},'
                f'{self._fused_y:.3f},'
                f'{self._fused_z:.3f}) '
                f'yaw={self._fused_yaw:.3f} '
                f'alpha_gps={self._gps_alpha:.2f} '
                f'alpha_odom={self._odom_alpha:.2f}'
            )
            self._diag_pub.publish(diag)


def _blend(current: float, measured: float, alpha: float) -> float:
    return alpha * measured + (1.0 - alpha) * current


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def main(args=None) -> None:
    """Entry point for the complementary_filter_node executable."""
    rclpy.init(args=args)
    node = ComplementaryFilterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
