"""Reactive lidar controller that steers away from obstacles instead of only stopping."""

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarObstacleAvoid(Node):
    """Publish cmd_vel that combines obstacle avoidance steering with forward motion."""

    def __init__(self) -> None:
        super().__init__('lidar_obstacle_avoid')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('avoid_distance', 1.2)
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('front_angle_degrees', 70.0)
        self.declare_parameter('publish_rate', 10.0)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self._left_min: float = math.inf
        self._right_min: float = math.inf
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('Started lidar obstacle avoid controller')

    def scan_callback(self, scan: LaserScan) -> None:
        """Split the front sector into left/right and cache each side's minimum range."""
        half_angle = math.radians(
            float(self.get_parameter('front_angle_degrees').value)
        ) * 0.5
        left_ranges: List[float] = []
        right_ranges: List[float] = []
        for index, reading in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > half_angle:
                continue
            if not math.isfinite(reading):
                continue
            if angle >= 0.0:
                left_ranges.append(reading)
            else:
                right_ranges.append(reading)
        self._left_min = min(left_ranges) if left_ranges else math.inf
        self._right_min = min(right_ranges) if right_ranges else math.inf

    def tick(self) -> None:
        """Publish a velocity command that avoids obstacles by steering away from them."""
        command = Twist()
        forward_speed = float(self.get_parameter('forward_speed').value)
        avoid_distance = float(self.get_parameter('avoid_distance').value)
        stop_distance = float(self.get_parameter('stop_distance').value)
        turn_speed = float(self.get_parameter('turn_speed').value)

        nearest = min(self._left_min, self._right_min)
        left_is_closer = self._left_min <= self._right_min
        turn_direction = -1.0 if left_is_closer else 1.0

        if nearest <= stop_distance:
            command.linear.x = 0.0
            command.angular.z = turn_direction * turn_speed
        elif nearest <= avoid_distance:
            span = avoid_distance - stop_distance
            factor = max(0.0, min(1.0, (nearest - stop_distance) / span))
            command.linear.x = factor * forward_speed
            command.angular.z = turn_direction * turn_speed * (1.0 - factor)
        else:
            command.linear.x = forward_speed

        self.publisher.publish(command)


def main(args=None) -> None:
    """Initialise rclpy, spin the LidarObstacleAvoid node, and shut down cleanly."""
    rclpy.init(args=args)
    node = LidarObstacleAvoid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
