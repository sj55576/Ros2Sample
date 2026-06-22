"""Telemetry aggregator that logs flight statistics and publishes diagnostic summaries."""

from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from drone_sim.telemetry_utils import compute_distance, compute_speed, format_telemetry


class TelemetryLogger(Node):

    def __init__(self) -> None:
        super().__init__('telemetry_logger')

        self.declare_parameter('log_interval_sec', 5.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._total_distance_m: float = 0.0
        self._max_speed_ms: float = 0.0
        self._max_altitude_m: float = 0.0
        self._min_battery_pct: float = 100.0
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None
        self._last_z: Optional[float] = None
        self._flight_start_time = None
        self._battery_pct: float = 100.0

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        self._summary_pub = self.create_publisher(String, 'telemetry_summary', 10)

        self.create_timer(log_interval_sec, self._on_log_timer)

        publish_period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(publish_period, self._on_publish_timer)

        self.get_logger().info(
            f'TelemetryLogger started: log_interval={log_interval_sec:.1f}s, '
            f'publish_rate={publish_rate_hz:.1f}Hz'
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self._flight_start_time is None:
            self._flight_start_time = self.get_clock().now()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        speed = compute_speed(vx, vy, vz)

        self._max_speed_ms = max(self._max_speed_ms, speed)
        self._max_altitude_m = max(self._max_altitude_m, z)

        if self._last_x is not None:
            self._total_distance_m += compute_distance(
                self._last_x, self._last_y, self._last_z, x, y, z
            )

        self._last_x = x
        self._last_y = y
        self._last_z = z

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_pct = msg.percentage * 100.0
        self._min_battery_pct = min(self._min_battery_pct, self._battery_pct)

    def _flight_time_sec(self) -> float:
        if self._flight_start_time is None:
            return 0.0
        return (self.get_clock().now() - self._flight_start_time).nanoseconds * 1e-9

    def _on_log_timer(self) -> None:
        flight_time = self._flight_time_sec()
        self.get_logger().info(
            f'Telemetry: dist={self._total_distance_m:.1f}m '
            f'max_alt={self._max_altitude_m:.1f}m '
            f'max_spd={self._max_speed_ms:.1f}m/s '
            f'bat={self._battery_pct:.0f}% '
            f'flight_time={flight_time:.0f}s'
        )

    def _on_publish_timer(self) -> None:
        current_x = self._last_x if self._last_x is not None else 0.0
        current_y = self._last_y if self._last_y is not None else 0.0
        current_z = self._last_z if self._last_z is not None else 0.0

        payload = format_telemetry(
            total_distance_m=self._total_distance_m,
            max_altitude_m=self._max_altitude_m,
            max_speed_ms=self._max_speed_ms,
            battery_pct=self._battery_pct,
            flight_time_sec=self._flight_time_sec(),
            current_x=current_x,
            current_y=current_y,
            current_z=current_z,
        )

        msg = String()
        msg.data = payload
        self._summary_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
