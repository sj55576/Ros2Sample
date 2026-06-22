"""Wind disturbance generator that publishes a time-varying wind velocity vector."""

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from drone_sim.wind_utils import compute_wind


class WindDisturbance(Node):
    """Simulate time-varying wind and publish the resulting velocity vector."""

    def __init__(self) -> None:
        super().__init__('wind_disturbance')

        self.declare_parameter('base_wind_x', 0.5)
        self.declare_parameter('base_wind_y', 0.0)
        self.declare_parameter('base_wind_z', 0.0)
        self.declare_parameter('gust_amplitude', 0.3)
        self.declare_parameter('gust_period_sec', 8.0)
        self.declare_parameter('turbulence_intensity', 0.1)
        self.declare_parameter('publish_rate_hz', 10.0)

        self._base = (
            float(self.get_parameter('base_wind_x').value),
            float(self.get_parameter('base_wind_y').value),
            float(self.get_parameter('base_wind_z').value),
        )
        self._gust_amplitude = float(self.get_parameter('gust_amplitude').value)
        self._gust_period_sec = float(self.get_parameter('gust_period_sec').value)
        self._turbulence_intensity = float(self.get_parameter('turbulence_intensity').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._start_time = self.get_clock().now()
        self._last_log_sec = 0.0

        self._wind_pub = self.create_publisher(Vector3, 'wind_velocity', 10)

        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'WindDisturbance started: base=({self._base[0]:.2f}, {self._base[1]:.2f}, '
            f'{self._base[2]:.2f}) m/s, gust_amplitude={self._gust_amplitude:.2f} m/s, '
            f'gust_period={self._gust_period_sec:.1f} s, '
            f'turbulence={self._turbulence_intensity:.2f} m/s'
        )

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        wx, wy, wz = compute_wind(
            self._base,
            self._gust_amplitude,
            self._gust_period_sec,
            self._turbulence_intensity,
            elapsed,
        )

        msg = Vector3()
        msg.x = wx
        msg.y = wy
        msg.z = wz
        self._wind_pub.publish(msg)

        if elapsed - self._last_log_sec >= 10.0:
            self.get_logger().info(
                f'Wind: x={wx:.3f}, y={wy:.3f}, z={wz:.3f} m/s'
            )
            self._last_log_sec = elapsed


def main(args=None) -> None:
    """Entry point for the wind_disturbance executable."""
    rclpy.init(args=args)
    node = WindDisturbance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
