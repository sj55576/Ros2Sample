"""Publish a repeating list of PoseStamped waypoints for a simulated drone."""

from math import dist

from drone_sim.waypoint_utils import parse_waypoints, Point3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class WaypointCommander(Node):
    """Cycle through configured waypoints when odometry is within tolerance."""

    def __init__(self) -> None:
        super().__init__('waypoint_commander')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('tolerance_m', 0.25)
        self.declare_parameter('hold_time_sec', 1.0)
        self.declare_parameter('loop', True)
        self.declare_parameter(
            'waypoints',
            [
                0.0, 0.0, 1.0,
                2.0, 0.0, 1.5,
                2.0, 2.0, 1.0,
                0.0, 2.0, 1.2,
                0.0, 0.0, 1.0,
            ],
        )

        self.frame_id = self.get_parameter('frame_id').value
        self.tolerance_m = float(self.get_parameter('tolerance_m').value)
        self.hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.waypoints = parse_waypoints(self.get_parameter('waypoints').value)
        self.current_index = 0
        self.current_position: Point3 = (0.0, 0.0, 0.0)
        self.arrival_time = None

        self.setpoint_pub = self.create_publisher(PoseStamped, 'setpoint_pose', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._publish_setpoint)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint(s)')

    def _on_odom(self, msg: Odometry) -> None:
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def _publish_setpoint(self) -> None:
        target = self.waypoints[self.current_index]
        now = self.get_clock().now()
        if dist(self.current_position, target) <= self.tolerance_m:
            if self.arrival_time is None:
                self.arrival_time = now
                self.get_logger().info(f'Reached waypoint {self.current_index}: {target}')
            elif (now - self.arrival_time).nanoseconds * 1e-9 >= self.hold_time_sec:
                self._advance_waypoint()
                target = self.waypoints[self.current_index]
        else:
            self.arrival_time = None

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def _advance_waypoint(self) -> None:
        self.arrival_time = None
        if self.current_index + 1 < len(self.waypoints):
            self.current_index += 1
        elif self.loop:
            self.current_index = 0
        self.get_logger().info(
            f'Commanding waypoint {self.current_index}: {self.waypoints[self.current_index]}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
