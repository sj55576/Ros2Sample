"""Battery simulation node that models energy consumption and publishes battery state."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class BatteryMonitor(Node):
    """Simulate battery drain and publish battery state based on motor activity."""

    def __init__(self) -> None:
        super().__init__('battery_monitor')

        self.declare_parameter('capacity_wh', 50.0)
        self.declare_parameter('idle_power_w', 5.0)
        self.declare_parameter('motor_power_w', 80.0)
        self.declare_parameter('critical_pct', 15.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        self._capacity_wh = float(self.get_parameter('capacity_wh').value)
        self._idle_power_w = float(self.get_parameter('idle_power_w').value)
        self._motor_power_w = float(self.get_parameter('motor_power_w').value)
        self._critical_pct = float(self.get_parameter('critical_pct').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._remaining_wh = self._capacity_wh
        self._throttle = 0.0
        self._critical_warned = False
        self._last_tick = self.get_clock().now()

        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self._low_battery_pub = self.create_publisher(Bool, 'low_battery', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'BatteryMonitor started: capacity={self._capacity_wh:.1f} Wh, '
            f'idle={self._idle_power_w:.1f} W, motor={self._motor_power_w:.1f} W, '
            f'critical={self._critical_pct:.1f}%'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = abs(msg.linear.x)
        vy = abs(msg.linear.y)
        vz = abs(msg.linear.z)
        wz = abs(msg.angular.z)
        self._throttle = min(1.0, (vx + vy + vz + wz) / 4.0)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt_sec = max((now - self._last_tick).nanoseconds * 1e-9, 1e-4)
        self._last_tick = now

        power_w = self._idle_power_w + self._throttle * self._motor_power_w
        drain_wh = power_w * (dt_sec / 3600.0)
        self._remaining_wh = max(0.0, self._remaining_wh - drain_wh)

        pct = self._remaining_wh / self._capacity_wh if self._capacity_wh > 0.0 else 0.0
        pct_display = pct * 100.0

        if pct_display <= self._critical_pct and not self._critical_warned:
            self.get_logger().warn(
                f'Battery critical: {pct_display:.1f}% remaining!'
            )
            self._critical_warned = True

        voltage = 10.0 + pct * 2.6
        current = -(power_w / voltage) if voltage > 0.0 else 0.0

        state = BatteryState()
        state.header.stamp = now.to_msg()
        state.voltage = voltage
        state.current = current
        state.charge = self._remaining_wh
        state.capacity = self._capacity_wh
        state.design_capacity = self._capacity_wh
        state.percentage = float(pct)
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        state.present = True
        self._battery_pub.publish(state)

        low = Bool()
        low.data = pct_display <= self._critical_pct
        self._low_battery_pub.publish(low)


def main(args=None) -> None:
    """Entry point for the battery_monitor executable."""
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
