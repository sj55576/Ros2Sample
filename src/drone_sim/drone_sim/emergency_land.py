"""Emergency landing node that descends safely on low battery or manual trigger."""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class EmergencyLand(Node):
    """Monitor battery and altitude, then command a safe descent when triggered."""

    def __init__(self) -> None:
        super().__init__('emergency_land')

        self.declare_parameter('descent_speed', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._descent_speed = float(self.get_parameter('descent_speed').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._landing = False
        self._altitude = 0.0

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'low_battery', self._on_low_battery, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_service(Trigger, 'emergency_land', self._on_trigger)

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'EmergencyLand ready: descent_speed={self._descent_speed:.2f} m/s'
        )

    def _on_low_battery(self, msg: Bool) -> None:
        if msg.data and not self._landing:
            self.get_logger().warn('Low battery received — initiating emergency landing.')
            self._landing = True

    def _on_odom(self, msg: Odometry) -> None:
        self._altitude = msg.pose.pose.position.z

    def _on_trigger(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._landing:
            response.success = True
            response.message = 'Already landing'
        else:
            self._landing = True
            self.get_logger().warn('Emergency landing triggered via service.')
            response.success = True
            response.message = 'Emergency landing initiated'
        return response

    def _tick(self) -> None:
        if not self._landing:
            return

        cmd = Twist()
        if self._altitude > 0.05:
            cmd.linear.z = -self._descent_speed
        else:
            self.get_logger().info('Landed safely.')
            self._landing = False
        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    """Entry point for the emergency_land executable."""
    rclpy.init(args=args)
    node = EmergencyLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
