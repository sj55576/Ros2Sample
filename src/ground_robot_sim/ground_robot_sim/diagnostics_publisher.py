"""Publish ground robot health as diagnostic_msgs/DiagnosticArray."""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class GroundRobotDiagnosticsPublisher(Node):
    """Aggregate ground robot sensor data and publish standardized diagnostics."""

    def __init__(self) -> None:
        super().__init__('ground_robot_diagnostics')

        self.declare_parameter('robot_name', 'ground_robot')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('min_scan_warn_m', 0.3)
        self.declare_parameter('speed_warn_ms', 0.7)

        self._robot_name = str(self.get_parameter('robot_name').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._min_scan_warn = float(self.get_parameter('min_scan_warn_m').value)
        self._speed_warn = float(self.get_parameter('speed_warn_ms').value)

        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._linear_speed: float = 0.0
        self._angular_speed: float = 0.0
        self._min_scan_range: float = float('inf')
        self._odom_received: bool = False
        self._scan_received: bool = False

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)

        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10,
        )

        period = 1.0 / max(publish_rate, 0.1)
        self.create_timer(period, self._publish_diagnostics)

        self.get_logger().info(
            f'GroundRobotDiagnosticsPublisher started for {self._robot_name}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._linear_speed = abs(msg.twist.twist.linear.x)
        self._angular_speed = abs(msg.twist.twist.angular.z)
        q = msg.pose.pose.orientation
        self._yaw = 2.0 * math.atan2(q.z, q.w)
        self._odom_received = True

    def _on_scan(self, msg: LaserScan) -> None:
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min <= r <= msg.range_max
        ]
        self._min_scan_range = min(valid_ranges) if valid_ranges else float('inf')
        self._scan_received = True

    def _publish_diagnostics(self) -> None:
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        diag_array.status.append(self._proximity_status())
        diag_array.status.append(self._motion_status())
        diag_array.status.append(self._position_status())

        self._diag_pub.publish(diag_array)

    def _proximity_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'{self._robot_name}/proximity'
        status.hardware_id = self._robot_name

        if not self._scan_received:
            status.level = DiagnosticStatus.STALE
            status.message = 'No scan data received'
            return status

        if self._min_scan_range < self._min_scan_warn:
            status.level = DiagnosticStatus.WARN
            status.message = f'Object nearby: {self._min_scan_range:.2f} m'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Clear: nearest {self._min_scan_range:.2f} m'

        status.values = [
            KeyValue(key='min_range_m', value=f'{self._min_scan_range:.3f}'),
        ]
        return status

    def _motion_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'{self._robot_name}/motion'
        status.hardware_id = self._robot_name

        if not self._odom_received:
            status.level = DiagnosticStatus.STALE
            status.message = 'No odometry data received'
            return status

        if self._linear_speed > self._speed_warn:
            status.level = DiagnosticStatus.WARN
            status.message = f'High speed: {self._linear_speed:.2f} m/s'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Speed: {self._linear_speed:.2f} m/s'

        status.values = [
            KeyValue(key='linear_speed_ms', value=f'{self._linear_speed:.3f}'),
            KeyValue(key='angular_speed_rads', value=f'{self._angular_speed:.3f}'),
        ]
        return status

    def _position_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'{self._robot_name}/position'
        status.hardware_id = self._robot_name

        if not self._odom_received:
            status.level = DiagnosticStatus.STALE
            status.message = 'No position data received'
            return status

        status.level = DiagnosticStatus.OK
        status.message = f'({self._x:.2f}, {self._y:.2f})'
        status.values = [
            KeyValue(key='x', value=f'{self._x:.3f}'),
            KeyValue(key='y', value=f'{self._y:.3f}'),
            KeyValue(key='heading_rad', value=f'{self._yaw:.3f}'),
        ]
        return status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundRobotDiagnosticsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
