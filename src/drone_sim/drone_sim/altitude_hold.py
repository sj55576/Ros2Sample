"""A tiny altitude-hold controller that commands vertical velocity."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_sim.math_utils import clamp


class AltitudeHold(Node):
    """Subscribe to odometry and publish cmd_vel commands to hold a target altitude."""

    def __init__(self) -> None:
        super().__init__('altitude_hold')
        self.declare_parameter('target_altitude_m', 2.0)
        self.declare_parameter('kp', 1.3)
        self.declare_parameter('max_vertical_speed', 1.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.target_altitude_m = float(self.get_parameter('target_altitude_m').value)
        self.kp = float(self.get_parameter('kp').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.current_altitude_m = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._publish_command)
        self.get_logger().info(f'Holding altitude at {self.target_altitude_m:.2f} m')

    def _on_odom(self, msg: Odometry) -> None:
        self.current_altitude_m = msg.pose.pose.position.z

    def _publish_command(self) -> None:
        error = self.target_altitude_m - self.current_altitude_m
        cmd = Twist()
        cmd.linear.z = clamp(error * self.kp, -self.max_vertical_speed, self.max_vertical_speed)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AltitudeHold()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
