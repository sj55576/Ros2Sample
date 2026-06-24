"""Pure Pursuit path follower for learning purposes."""

import math
from math import atan2
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class SimplePathFollower(Node):
    """Follow a nav_msgs/Path using the Pure Pursuit algorithm."""

    def __init__(self) -> None:
        super().__init__('simple_path_follower')
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('control_rate', 20.0)

        self._path: Optional[Path] = None
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._goal_reached: bool = False
        self._log_counter: int = 0

        self.create_subscription(Path, '/plan', self._path_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info('パスフォロワー起動: パスを待っています...')

    def _path_callback(self, msg: Path) -> None:
        """Store the new path and reset goal-reached state."""
        self._path = msg
        self._goal_reached = False
        self.get_logger().info(f'パス受信: {len(msg.poses)}ポーズ')

    def _odom_callback(self, msg: Odometry) -> None:
        """Update cached robot pose from odometry."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = 2.0 * atan2(q.z, q.w)

    def _control_tick(self) -> None:
        """Compute and publish the next velocity command via Pure Pursuit."""
        cmd = Twist()

        if self._path is None or len(self._path.poses) == 0:
            self._cmd_pub.publish(cmd)
            return

        if self._goal_reached:
            self._cmd_pub.publish(cmd)
            return

        goal_pose = self._path.poses[-1].pose
        gdx = goal_pose.position.x - self._x
        gdy = goal_pose.position.y - self._y
        goal_dist = math.sqrt(gdx * gdx + gdy * gdy)

        goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        if goal_dist < goal_tolerance:
            self._goal_reached = True
            self._cmd_pub.publish(cmd)
            self.get_logger().info('ゴールに到達しました')
            return

        lookahead = float(self.get_parameter('lookahead_distance').value)
        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)

        lookahead_pose = None
        lookahead_index = len(self._path.poses) - 1
        for i, pose_stamped in enumerate(self._path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dx = px - self._x
            dy = py - self._y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist >= lookahead:
                lookahead_pose = pose_stamped.pose
                lookahead_index = i
                break

        if lookahead_pose is None:
            lookahead_pose = self._path.poses[-1].pose

        lx = lookahead_pose.position.x - self._x
        ly = lookahead_pose.position.y - self._y
        alpha = atan2(ly, lx) - self._yaw
        # Wrap alpha to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        curvature = 2.0 * math.sin(alpha) / max(lookahead, 1e-6)

        # Reduce speed as we approach the goal
        speed_factor = min(1.0, goal_dist / (lookahead * 2.0))
        linear_vel = max_linear * speed_factor

        angular_vel = linear_vel * curvature
        angular_vel = max(-max_angular, min(max_angular, angular_vel))

        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self._cmd_pub.publish(cmd)

        # Periodic status log (every 20 ticks)
        self._log_counter += 1
        if self._log_counter >= 20:
            self._log_counter = 0
            self.get_logger().info(
                f'ゴールまで: {goal_dist:.2f}m, 現在のウェイポイント: {lookahead_index}'
            )


def main(args=None) -> None:
    """Initialise rclpy, spin the SimplePathFollower node, and shut down cleanly."""
    rclpy.init(args=args)
    node = SimplePathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
