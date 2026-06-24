"""Collision avoidance node that adjusts drone setpoints using potential fields."""

from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_sim.collision_utils import apply_avoidance, compute_repulsive_force


class CollisionAvoidance(Node):
    """Subscribe to multiple drone odom topics and publish adjusted setpoints."""

    def __init__(self) -> None:
        super().__init__('collision_avoidance')

        self.declare_parameter('drone_odom_topics', ['/drone_1/odom', '/drone_2/odom'])
        self.declare_parameter('own_odom_topic', '/drone_1/odom')
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('influence_distance', 3.0)
        self.declare_parameter('repulsion_gain', 1.5)
        self.declare_parameter('max_adjustment', 2.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'odom')

        drone_topics = list(self.get_parameter('drone_odom_topics').value)
        self._own_topic = str(self.get_parameter('own_odom_topic').value)
        self._safety_dist = float(self.get_parameter('safety_distance').value)
        self._influence_dist = float(self.get_parameter('influence_distance').value)
        self._gain = float(self.get_parameter('repulsion_gain').value)
        self._max_adj = float(self.get_parameter('max_adjustment').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        self._positions: Dict[str, Tuple[float, float, float]] = {}
        self._own_pos: Optional[Tuple[float, float, float]] = None
        self._current_setpoint: Optional[Tuple[float, float, float]] = None

        for topic in drone_topics:
            self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self._on_odom(t, msg),
                10,
            )

        self.create_subscription(
            PoseStamped, 'setpoint_pose', self._on_setpoint, 10,
        )

        self._adjusted_pub = self.create_publisher(
            PoseStamped, 'adjusted_setpoint_pose', 10,
        )

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'CollisionAvoidance started: tracking {len(drone_topics)} drones, '
            f'safety={self._safety_dist}m, influence={self._influence_dist}m'
        )

    def _on_odom(self, topic: str, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._positions[topic] = (pos.x, pos.y, pos.z)
        if topic == self._own_topic:
            self._own_pos = (pos.x, pos.y, pos.z)

    def _on_setpoint(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._current_setpoint = (p.x, p.y, p.z)

    def _tick(self) -> None:
        if self._own_pos is None or self._current_setpoint is None:
            return

        others = [
            pos for topic, pos in self._positions.items()
            if topic != self._own_topic
        ]

        if not others:
            # No other drones; pass through setpoint unchanged
            self._publish_setpoint(self._current_setpoint)
            return

        repulsive = compute_repulsive_force(
            self._own_pos, others,
            self._safety_dist, self._influence_dist, self._gain,
        )

        adjusted = apply_avoidance(
            self._current_setpoint, repulsive, self._max_adj,
        )

        self._publish_setpoint(adjusted)

    def _publish_setpoint(self, target: Tuple[float, float, float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = max(0.0, target[2])
        msg.pose.orientation.w = 1.0
        self._adjusted_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
