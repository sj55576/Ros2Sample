"""Build a 2D occupancy grid online from odometry and laser scans (SLAM入門)."""

import math
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from nav2_learning.mapping_utils import integrate_scan, log_odds_to_occupancy


class SimpleOccupancyMapper(Node):
    """Incrementally build an OccupancyGrid from LaserScan and Odometry using log-odds.

    Odometry is treated as ground truth (map -> odom is a static identity
    transform), so this node demonstrates the mapping half of SLAM without
    localization, loop closure, or scan matching.
    """

    def __init__(self) -> None:
        super().__init__('simple_occupancy_mapper')
        self.declare_parameter('map_width', 220)
        self.declare_parameter('map_height', 220)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', -5.5)
        self.declare_parameter('origin_y', -5.5)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('hit_log_odds', 0.85)
        self.declare_parameter('miss_log_odds', -0.4)
        self.declare_parameter('log_odds_min', -4.0)
        self.declare_parameter('log_odds_max', 4.0)
        self.declare_parameter('occupied_threshold', 0.65)
        self.declare_parameter('free_threshold', 0.35)
        # NOTE: ground_robot_node casts its simulated rays from the base
        # position (self.x, self.y) even though TF places base_scan 0.18m
        # forward of base_link, so the default offset here is 0.0 to match
        # the simulated sensor's true origin.
        self.declare_parameter('laser_offset_x', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('scan_throttle', 1)

        self._width = int(self.get_parameter('map_width').value)
        self._height = int(self.get_parameter('map_height').value)
        self._resolution = float(self.get_parameter('resolution').value)
        self._origin_x = float(self.get_parameter('origin_x').value)
        self._origin_y = float(self.get_parameter('origin_y').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._scan_throttle = max(1, int(self.get_parameter('scan_throttle').value))

        self._log_odds: List[float] = [0.0] * (self._width * self._height)
        self._have_odom = False
        self._logged_no_odom = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._scan_counter = 0

        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_map_odom_tf()

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._publish_map)

        coverage_x = self._width * self._resolution
        coverage_y = self._height * self._resolution
        self.get_logger().info(
            f'占有格子地図マッピング開始: {self._width}x{self._height}セル, '
            f'解像度={self._resolution}m/cell, '
            f'カバー範囲={coverage_x:.1f}m x {coverage_y:.1f}m'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """Update cached robot pose (x, y, yaw) from odometry."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = 2.0 * math.atan2(q.z, q.w)
        self._have_odom = True

    def _scan_callback(self, msg: LaserScan) -> None:
        """Integrate a laser scan into the log-odds grid, throttled by scan_throttle."""
        self._scan_counter += 1
        if (self._scan_counter - 1) % self._scan_throttle != 0:
            return

        if not self._have_odom:
            if not self._logged_no_odom:
                self.get_logger().info('オドメトリ未受信のためスキャンを無視します')
                self._logged_no_odom = True
            return

        laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        sensor_x = self._x + laser_offset_x * math.cos(self._yaw)
        sensor_y = self._y + laser_offset_x * math.sin(self._yaw)
        sensor_yaw = self._yaw

        integrate_scan(
            self._log_odds,
            self._width,
            self._height,
            self._origin_x,
            self._origin_y,
            self._resolution,
            sensor_x,
            sensor_y,
            sensor_yaw,
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            hit_log_odds=float(self.get_parameter('hit_log_odds').value),
            miss_log_odds=float(self.get_parameter('miss_log_odds').value),
            log_odds_min=float(self.get_parameter('log_odds_min').value),
            log_odds_max=float(self.get_parameter('log_odds_max').value),
        )

    def _broadcast_map_odom_tf(self) -> None:
        """Broadcast a static identity transform from map to odom.

        Odometry is treated as ground truth in this demo: there is no
        localization or loop closure, so map and odom frames never diverge.
        """
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._map_frame
        tf.child_frame_id = 'odom'
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)

    def _build_map_msg(self) -> OccupancyGrid:
        """Assemble an OccupancyGrid message from the current log-odds grid."""
        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = self._resolution
        msg.info.width = self._width
        msg.info.height = self._height
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        occupied_threshold = float(self.get_parameter('occupied_threshold').value)
        free_threshold = float(self.get_parameter('free_threshold').value)
        msg.data = log_odds_to_occupancy(
            self._log_odds,
            occupied_threshold=occupied_threshold,
            free_threshold=free_threshold,
        )
        return msg

    def _publish_map(self) -> None:
        """Build the latest OccupancyGrid message and publish it."""
        msg = self._build_map_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(msg)


def main(args=None) -> None:
    """Initialise rclpy, spin the SimpleOccupancyMapper node, and shut down cleanly."""
    rclpy.init(args=args)
    node = SimpleOccupancyMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
