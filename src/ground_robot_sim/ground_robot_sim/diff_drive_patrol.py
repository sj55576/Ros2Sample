"""Open-loop square patrol controller for the sample ground robot."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class DiffDrivePatrol(Node):
    """Publish cmd_vel commands that drive repeated forward and turn phases."""

    def __init__(self) -> None:
        super().__init__('diff_drive_patrol')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('turn_speed', 0.7)
        self.declare_parameter('forward_duration', 5.0)
        self.declare_parameter('turn_duration', 2.25)
        self.declare_parameter('pause_duration', 0.4)
        self.declare_parameter('publish_rate', 10.0)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.phase_started = self.get_clock().now()
        self.phase = 'forward'
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('Started open-loop diff-drive patrol controller')

    def tick(self) -> None:
        """Advance the patrol state machine and publish the current command."""
        elapsed = (self.get_clock().now() - self.phase_started).nanoseconds * 1e-9
        forward_duration = float(self.get_parameter('forward_duration').value)
        turn_duration = float(self.get_parameter('turn_duration').value)
        pause_duration = float(self.get_parameter('pause_duration').value)

        if self.phase == 'forward' and elapsed >= forward_duration:
            self.switch_phase('pause_before_turn')
        elif self.phase == 'pause_before_turn' and elapsed >= pause_duration:
            self.switch_phase('turn')
        elif self.phase == 'turn' and elapsed >= turn_duration:
            self.switch_phase('pause_before_forward')
        elif self.phase == 'pause_before_forward' and elapsed >= pause_duration:
            self.switch_phase('forward')

        command = Twist()
        if self.phase == 'forward':
            command.linear.x = float(self.get_parameter('forward_speed').value)
        elif self.phase == 'turn':
            command.angular.z = float(self.get_parameter('turn_speed').value)
        self.publisher.publish(command)

    def switch_phase(self, phase: str) -> None:
        """Record a phase transition with fresh timing."""
        self.phase = phase
        self.phase_started = self.get_clock().now()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiffDrivePatrol()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
