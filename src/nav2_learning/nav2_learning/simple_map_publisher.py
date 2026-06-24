"""Publish a procedurally generated OccupancyGrid map for Nav2 learning."""

import math
from typing import List, Tuple

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster

from nav2_learning.map_utils import (
    create_empty_grid,
    draw_filled_circle,
    draw_walls,
    inflate_obstacles,
    world_to_grid,
)


def _parse_obstacles(raw: List[float]) -> List[Tuple[float, float, float]]:
    """Parse a flat [cx, cy, r, ...] list into (cx, cy, radius) tuples."""
    if len(raw) % 3 != 0:
        raise ValueError('obstacles parameter length must be a multiple of 3')
    values = [float(v) for v in raw]
    return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)]


class SimpleMapPublisher(Node):
    """Generate and publish a static procedural OccupancyGrid for Nav2 learning."""

    def __init__(self) -> None:
        super().__init__('simple_map_publisher')
        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', -2.5)
        self.declare_parameter('origin_y', -2.5)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('wall_obstacles', True)
        self.declare_parameter(
            'obstacles',
            [1.0, 0.0, 0.5, -1.0, 1.0, 0.4, 0.5, -1.5, 0.3],
        )
        self.declare_parameter('inflation_radius_cells', 3)

        width = int(self.get_parameter('map_width').value)
        height = int(self.get_parameter('map_height').value)
        resolution = float(self.get_parameter('resolution').value)
        origin_x = float(self.get_parameter('origin_x').value)
        origin_y = float(self.get_parameter('origin_y').value)
        wall_obstacles = bool(self.get_parameter('wall_obstacles').value)
        inflation_radius = int(self.get_parameter('inflation_radius_cells').value)
        raw_obstacles = list(self.get_parameter('obstacles').value)

        grid = create_empty_grid(width, height)

        if wall_obstacles:
            draw_walls(grid, width, height)

        obstacles = _parse_obstacles(raw_obstacles)
        for cx_w, cy_w, r_w in obstacles:
            cx_g, cy_g = world_to_grid(cx_w, cy_w, origin_x, origin_y, resolution)
            r_g = max(1, int(math.ceil(r_w / resolution)))
            draw_filled_circle(grid, width, height, cx_g, cy_g, r_g)

        if inflation_radius > 0:
            grid = inflate_obstacles(grid, width, height, inflation_radius)

        self._map_msg = self._build_map_msg(grid, width, height, resolution, origin_x, origin_y)

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
        self._publish_map()

        self.get_logger().info(
            f'マップ生成完了: {width}x{height}セル, '
            f'解像度={resolution}m/cell, '
            f'障害物数={len(obstacles)}'
        )

    def _build_map_msg(
        self,
        grid: List[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> OccupancyGrid:
        """Assemble an OccupancyGrid message from the generated grid data."""
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in grid]
        return msg

    def _broadcast_map_odom_tf(self) -> None:
        """Broadcast a static identity transform from map to odom."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)

    def _publish_map(self) -> None:
        """Update the map timestamp and publish."""
        self._map_msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(self._map_msg)


def main(args=None) -> None:
    """Initialise rclpy, spin the SimpleMapPublisher node, and shut down cleanly."""
    rclpy.init(args=args)
    node = SimpleMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
