"""Closed-loop go-to-goal waypoint follower for the sample ground robot."""

import math
from math import atan2
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from ground_robot_sim.geometry import normalize_angle
from ground_robot_sim.pid import PIDController
from nav_msgs.msg import Odometry
from rclpy.node import Node


def parse_waypoints_xy(raw: List[float]) -> List[Tuple[float, float]]:
    """Parse a flat [x1, y1, x2, y2, ...] list into (x, y) tuples.

    Raises ValueError when the list is empty, has odd length, or contains
    fewer than two values (i.e., no complete waypoint pair).
    """
    if len(raw) == 0 or len(raw) % 2 != 0:
        raise ValueError(
            'waypoints must be a non-empty flat list of (x, y) pairs '
            f'with even length; got length {len(raw)}'
        )
    return [(float(raw[i]), float(raw[i + 1])) for i in range(0, len(raw), 2)]


class WaypointFollower(Node):
    """Drive a diff-drive robot through a series of (x, y) waypoints via closed-loop control."""

    def __init__(self) -> None:
        super().__init__('waypoint_follower')
        self.declare_parameter(
            'waypoints',
            [1.5, 0.0, 1.5, 1.5, 0.0, 1.5, 0.0, 0.0],
        )
        self.declare_parameter('tolerance_m', 0.15)
        self.declare_parameter('hold_time_sec', 0.5)
        self.declare_parameter('loop', True)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.2)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('heading_gate_rad', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.waypoints = parse_waypoints_xy(
            list(self.get_parameter('waypoints').value)
        )
        self.current_index: int = 0
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.arrival_time = None
        self._completed_logged: bool = False

        kp_linear = float(self.get_parameter('kp_linear').value)
        ki_linear = float(self.get_parameter('ki_linear').value)
        kd_linear = float(self.get_parameter('kd_linear').value)
        kp_angular = float(self.get_parameter('kp_angular').value)
        ki_angular = float(self.get_parameter('ki_angular').value)
        kd_angular = float(self.get_parameter('kd_angular').value)
        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        self._linear_pid = PIDController(
            kp_linear, ki_linear, kd_linear,
            output_min=0.0, output_max=max_linear,
        )
        self._angular_pid = PIDController(
            kp_angular, ki_angular, kd_angular,
            output_min=-max_angular, output_max=max_angular,
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self.tick)
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoint(s); '
            f'starting at waypoint 0: {self.waypoints[0]}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """Update the cached pose from an odometry message."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = 2.0 * atan2(q.z, q.w)

    def tick(self) -> None:
        """Compute and publish the next cmd_vel based on the current pose."""
        command = Twist()

        tolerance_m = float(self.get_parameter('tolerance_m').value)
        hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        loop = bool(self.get_parameter('loop').value)
        heading_gate = float(self.get_parameter('heading_gate_rad').value)

        finished = self.current_index >= len(self.waypoints)
        if finished:
            if not self._completed_logged:
                self.get_logger().info('completed')
                self._completed_logged = True
            self.publisher.publish(command)
            return

        target_x, target_y = self.waypoints[self.current_index]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= tolerance_m:
            now = self.get_clock().now()
            if self.arrival_time is None:
                self.arrival_time = now
                self.get_logger().info(
                    f'Reached waypoint {self.current_index}: '
                    f'({target_x:.3f}, {target_y:.3f})'
                )
            elapsed = (now - self.arrival_time).nanoseconds * 1e-9
            if elapsed >= hold_time_sec:
                self._advance_waypoint(loop)
            self.publisher.publish(command)
            return

        self.arrival_time = None
        bearing = atan2(dy, dx)
        heading_error = normalize_angle(bearing - self.yaw)

        angular = self._angular_pid.compute(heading_error, self._dt)
        if abs(heading_error) <= heading_gate:
            linear = self._linear_pid.compute(distance, self._dt)
        else:
            linear = 0.0
            self._linear_pid.reset()

        command.linear.x = linear
        command.angular.z = angular
        self.publisher.publish(command)

    def _advance_waypoint(self, loop: bool) -> None:
        """Move to the next waypoint or wrap around when looping."""
        self.arrival_time = None
        self._linear_pid.reset()
        self._angular_pid.reset()
        next_index = self.current_index + 1
        if next_index < len(self.waypoints):
            self.current_index = next_index
        elif loop:
            self.current_index = 0
        else:
            self.current_index = len(self.waypoints)
            return
        self.get_logger().info(
            f'Commanding waypoint {self.current_index}: '
            f'{self.waypoints[self.current_index]}'
        )


def main(args=None) -> None:
    """Initialise rclpy, spin the WaypointFollower node, and shut down cleanly."""
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
