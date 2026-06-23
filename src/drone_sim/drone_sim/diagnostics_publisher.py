"""Publish drone health as diagnostic_msgs/DiagnosticArray."""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class DroneDiagnosticsPublisher(Node):
    """Aggregate drone sensor data and publish standardized diagnostics."""

    def __init__(self) -> None:
        super().__init__('drone_diagnostics')

        self.declare_parameter('robot_name', 'drone')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('battery_warn_pct', 30.0)
        self.declare_parameter('battery_error_pct', 15.0)
        self.declare_parameter('speed_warn_ms', 4.0)

        self._robot_name = str(self.get_parameter('robot_name').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._batt_warn = float(self.get_parameter('battery_warn_pct').value)
        self._batt_error = float(self.get_parameter('battery_error_pct').value)
        self._speed_warn = float(self.get_parameter('speed_warn_ms').value)

        self._battery_pct: float = 100.0
        self._voltage: float = 12.6
        self._x: float = 0.0
        self._y: float = 0.0
        self._z: float = 0.0
        self._speed: float = 0.0
        self._odom_received: bool = False
        self._battery_received: bool = False

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10,
        )

        period = 1.0 / max(publish_rate, 0.1)
        self.create_timer(period, self._publish_diagnostics)

        self.get_logger().info(
            f'DroneDiagnosticsPublisher started for {self._robot_name}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._z = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self._speed = (vx ** 2 + vy ** 2 + vz ** 2) ** 0.5
        self._odom_received = True

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_pct = msg.percentage * 100.0
        self._voltage = msg.voltage
        self._battery_received = True

    def _publish_diagnostics(self) -> None:
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        diag_array.status.append(self._battery_status())
        diag_array.status.append(self._motion_status())
        diag_array.status.append(self._position_status())

        self._diag_pub.publish(diag_array)

    def _battery_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'{self._robot_name}/battery'
        status.hardware_id = self._robot_name

        if not self._battery_received:
            status.level = DiagnosticStatus.STALE
            status.message = 'No battery data received'
            return status

        if self._battery_pct <= self._batt_error:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Battery critical: {self._battery_pct:.1f}%'
        elif self._battery_pct <= self._batt_warn:
            status.level = DiagnosticStatus.WARN
            status.message = f'Battery low: {self._battery_pct:.1f}%'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Battery OK: {self._battery_pct:.1f}%'

        status.values = [
            KeyValue(key='percentage', value=f'{self._battery_pct:.1f}'),
            KeyValue(key='voltage', value=f'{self._voltage:.2f}'),
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

        if self._speed > self._speed_warn:
            status.level = DiagnosticStatus.WARN
            status.message = f'High speed: {self._speed:.2f} m/s'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Speed: {self._speed:.2f} m/s'

        status.values = [
            KeyValue(key='speed_ms', value=f'{self._speed:.3f}'),
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
        status.message = f'({self._x:.2f}, {self._y:.2f}, {self._z:.2f})'
        status.values = [
            KeyValue(key='x', value=f'{self._x:.3f}'),
            KeyValue(key='y', value=f'{self._y:.3f}'),
            KeyValue(key='z', value=f'{self._z:.3f}'),
            KeyValue(key='altitude', value=f'{self._z:.3f}'),
        ]
        return status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneDiagnosticsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
