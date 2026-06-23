"""Geofence monitor that constrains drone flight to a configurable bounding box."""

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_sim.geofence_utils import check_boundary


class GeofenceMonitor(Node):
    """Monitor drone position and issue corrective setpoints when the geofence is breached."""

    def __init__(self) -> None:
        super().__init__('geofence_monitor')

        self.declare_parameter('boundary_min_x', -10.0)
        self.declare_parameter('boundary_max_x', 10.0)
        self.declare_parameter('boundary_min_y', -10.0)
        self.declare_parameter('boundary_max_y', 10.0)
        self.declare_parameter('boundary_min_z', 0.0)
        self.declare_parameter('boundary_max_z', 20.0)
        self.declare_parameter('margin_m', 1.0)
        self.declare_parameter('publish_rate_hz', 5.0)

        self._bounds_min = (
            float(self.get_parameter('boundary_min_x').value),
            float(self.get_parameter('boundary_min_y').value),
            float(self.get_parameter('boundary_min_z').value),
        )
        self._bounds_max = (
            float(self.get_parameter('boundary_max_x').value),
            float(self.get_parameter('boundary_max_y').value),
            float(self.get_parameter('boundary_max_z').value),
        )
        self._margin = float(self.get_parameter('margin_m').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._current_z: float = 0.0
        self._odom_received: bool = False
        self._last_odom: Optional[Odometry] = None

        self._breach_pub = self.create_publisher(Bool, 'geofence_breach', 10)
        self._setpoint_pub = self.create_publisher(PoseStamped, 'geofence_setpoint', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

        period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'GeofenceMonitor started: x=[{self._bounds_min[0]}, {self._bounds_max[0]}], '
            f'y=[{self._bounds_min[1]}, {self._bounds_max[1]}], '
            f'z=[{self._bounds_min[2]}, {self._bounds_max[2]}], '
            f'margin={self._margin} m'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._current_z = msg.pose.pose.position.z
        self._odom_received = True
        self._last_odom = msg

    def _tick(self) -> None:
        if not self._odom_received:
            return

        status, (cx, cy, cz) = check_boundary(
            self._current_x,
            self._current_y,
            self._current_z,
            self._bounds_min,
            self._bounds_max,
            self._margin,
        )

        breach_msg = Bool()
        breach_msg.data = status == 'breach'
        self._breach_pub.publish(breach_msg)

        if status == 'breach':
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = (
                self._last_odom.header.frame_id if self._last_odom is not None else 'odom'
            )
            sp.pose.position.x = cx
            sp.pose.position.y = cy
            sp.pose.position.z = cz
            sp.pose.orientation.w = 1.0
            self._setpoint_pub.publish(sp)
            self.get_logger().warn(
                f'Geofence breached at ({self._current_x:.2f}, {self._current_y:.2f}, '
                f'{self._current_z:.2f}); corrective setpoint ({cx:.2f}, {cy:.2f}, {cz:.2f})'
            )
        elif status == 'warning':
            self.get_logger().warn(
                f'Approaching geofence boundary at ({self._current_x:.2f}, '
                f'{self._current_y:.2f}, {self._current_z:.2f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeofenceMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
