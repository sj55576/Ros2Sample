"""Reactive sample that drives forward until lidar sees an obstacle."""

import math
from typing import List

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarObstacleStop(Node):
    """Publish a safe cmd_vel using only the latest front laser sector."""

    def __init__(self) -> None:
        super().__init__('lidar_obstacle_stop')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('stop_distance', 0.75)
        self.declare_parameter('front_angle_degrees', 30.0)
        self.declare_parameter('publish_rate', 10.0)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.front_ranges: List[float] = []
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('Started lidar obstacle stop controller')

    def scan_callback(self, scan: LaserScan) -> None:
        """Cache finite range values from a configurable sector around straight ahead."""
        half_angle = math.radians(float(self.get_parameter('front_angle_degrees').value)) * 0.5
        ranges = []
        for index, reading in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) <= half_angle and math.isfinite(reading):
                ranges.append(reading)
        self.front_ranges = ranges

    def tick(self) -> None:
        """Stop when blocked; otherwise command a slow forward crawl."""
        command = Twist()
        stop_distance = float(self.get_parameter('stop_distance').value)
        nearest = min(self.front_ranges) if self.front_ranges else math.inf
        if nearest > stop_distance:
            command.linear.x = float(self.get_parameter('forward_speed').value)
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarObstacleStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
