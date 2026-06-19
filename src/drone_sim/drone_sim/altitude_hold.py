"""A tiny altitude-hold controller that commands vertical velocity."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_sim.pid import PIDController


class AltitudeHold(Node):
    """Subscribe to odometry and publish cmd_vel commands to hold a target altitude."""

    def __init__(self) -> None:
        super().__init__('altitude_hold')
        self.declare_parameter('target_altitude_m', 2.0)
        self.declare_parameter('kp', 1.3)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('max_vertical_speed', 1.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.target_altitude_m = float(self.get_parameter('target_altitude_m').value)
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.current_altitude_m = 0.0
        publish_rate_hz = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self._dt = 1.0 / publish_rate_hz

        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            output_min=-self.max_vertical_speed,
            output_max=self.max_vertical_speed,
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(self._dt, self._publish_command)
        self.get_logger().info(f'Holding altitude at {self.target_altitude_m:.2f} m')

    def _on_odom(self, msg: Odometry) -> None:
        self.current_altitude_m = msg.pose.pose.position.z

    def _publish_command(self) -> None:
        error = self.target_altitude_m - self.current_altitude_m
        cmd = Twist()
        cmd.linear.z = self.pid.compute(error, self._dt)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AltitudeHold()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
