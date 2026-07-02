"""Reactive lidar controller that steers away from obstacles instead of only stopping."""

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
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
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.avoid_distance = float(
            self.get_parameter('avoid_distance').value
        )
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.tick)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info('Started lidar obstacle avoid controller')

    def _on_param_change(
        self, params: list,
    ) -> SetParametersResult:
        """Validate and apply dynamic parameter changes."""
        pending = {}
        for param in params:
            if param.name in ('stop_distance', 'avoid_distance'):
                val = float(param.value)
                if not math.isfinite(val) or val <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be a finite value > 0.0',
                    )
                pending[param.name] = val

        if pending:
            effective_stop = pending.get(
                'stop_distance', self.stop_distance
            )
            effective_avoid = pending.get(
                'avoid_distance', self.avoid_distance
            )
            if not effective_stop < effective_avoid:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        'stop_distance must remain strictly less than '
                        'avoid_distance (would be '
                        f'stop_distance={effective_stop}, '
                        f'avoid_distance={effective_avoid})'
                    ),
                )

        for param in params:
            if param.name == 'stop_distance':
                self.stop_distance = pending['stop_distance']
                self.get_logger().info(
                    f'stop_distance updated to {self.stop_distance:.3f}'
                )
            elif param.name == 'avoid_distance':
                self.avoid_distance = pending['avoid_distance']
                self.get_logger().info(
                    f'avoid_distance updated to {self.avoid_distance:.3f}'
                )

        return SetParametersResult(successful=True)

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
        avoid_distance = self.avoid_distance
        stop_distance = self.stop_distance
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
