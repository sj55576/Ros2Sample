"""Leader-follower formation controller that tracks a leader drone with a configurable offset."""

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_sim.formation_utils import compute_formation_target, smooth_position


class FormationController(Node):
    """Maintain a fixed offset from a leader drone by commanding a follower's setpoint."""

    def __init__(self) -> None:
        super().__init__('formation_controller')

        self.declare_parameter('leader_odom_topic', '/drone_1/odom')
        self.declare_parameter('offset_x', 2.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('smoothing_gain', 0.8)

        leader_odom_topic = self.get_parameter('leader_odom_topic').value
        self.offset_x = float(self.get_parameter('offset_x').value)
        self.offset_y = float(self.get_parameter('offset_y').value)
        self.offset_z = float(self.get_parameter('offset_z').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.smoothing_gain = float(self.get_parameter('smoothing_gain').value)

        self.smoothed_target: Optional[Tuple[float, float, float]] = None

        self.setpoint_pub = self.create_publisher(PoseStamped, 'setpoint_pose', 10)
        self.create_subscription(Odometry, leader_odom_topic, self._on_leader_odom, 10)

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._publish_setpoint)

    def _on_leader_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        new_target = compute_formation_target(
            pos.x, pos.y, pos.z,
            self.offset_x, self.offset_y, self.offset_z,
        )
        if self.smoothed_target is None:
            self.smoothed_target = new_target
            self.get_logger().info(
                f'Tracking leader; initial target {new_target}'
            )
        else:
            self.smoothed_target = smooth_position(
                self.smoothed_target, new_target, self.smoothing_gain
            )

    def _publish_setpoint(self) -> None:
        if self.smoothed_target is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.smoothed_target[0]
        msg.pose.position.y = self.smoothed_target[1]
        msg.pose.position.z = self.smoothed_target[2]
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
