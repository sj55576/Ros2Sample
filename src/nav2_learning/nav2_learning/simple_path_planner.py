"""A* path planner on OccupancyGrid for learning purposes."""

import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from nav2_learning.map_utils import (
    a_star_search,
    grid_to_world,
    is_valid_cell,
    world_to_grid,
)


class SimplePathPlanner(Node):
    """Plan paths on an OccupancyGrid using A* search."""

    def __init__(self) -> None:
        super().__init__('simple_path_planner')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('cost_threshold', 50)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('replan_rate', 0.0)

        self._map: Optional[OccupancyGrid] = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, latched_qos)
        self._plan_pub = self.create_publisher(Path, '/plan', 10)
        self.create_service(Trigger, '~/plan_path', self._plan_service_callback)

        replan_rate = float(self.get_parameter('replan_rate').value)
        if replan_rate > 0.0:
            self.create_timer(1.0 / replan_rate, self._plan_and_publish)

        self.get_logger().info('パスプランナー起動: マップを待っています...')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Store the received map and trigger initial planning."""
        self._map = msg
        self.get_logger().info(
            f'マップ受信: {msg.info.width}x{msg.info.height}セル'
        )
        replan_rate = float(self.get_parameter('replan_rate').value)
        if replan_rate <= 0.0:
            self._plan_and_publish()

    def _plan_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Handle on-demand replanning via service call."""
        if self._map is None:
            response.success = False
            response.message = 'マップがまだ受信されていません'
            return response
        path = self._plan_and_publish()
        if path is not None:
            response.success = True
            response.message = f'パス生成成功: {len(path.poses)}ポーズ'
        else:
            response.success = False
            response.message = 'パスの生成に失敗しました'
        return response

    def _plan_and_publish(self) -> Optional[Path]:
        """Run A* and publish the result; return the Path message or None on failure."""
        if self._map is None:
            return None

        start_x = float(self.get_parameter('start_x').value)
        start_y = float(self.get_parameter('start_y').value)
        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)
        cost_threshold = int(self.get_parameter('cost_threshold').value)
        allow_diagonal = bool(self.get_parameter('allow_diagonal').value)

        info = self._map.info
        width = info.width
        height = info.height
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        start = world_to_grid(start_x, start_y, ox, oy, resolution)
        goal = world_to_grid(goal_x, goal_y, ox, oy, resolution)

        if not is_valid_cell(start[0], start[1], width, height):
            self.get_logger().warn('スタート座標がマップ外です')
            return None
        if not is_valid_cell(goal[0], goal[1], width, height):
            self.get_logger().warn('ゴール座標がマップ外です')
            return None

        t0 = time.monotonic()
        grid_path = self._run_astar(
            start, goal, width, height, resolution, cost_threshold, allow_diagonal
        )
        elapsed = time.monotonic() - t0

        if grid_path is None:
            self.get_logger().warn('パスが見つかりませんでした')
            return None

        path_msg = self._build_path_msg(grid_path, ox, oy, resolution)
        self._plan_pub.publish(path_msg)
        self.get_logger().info(
            f'パス生成完了: {len(grid_path)}セル, '
            f'距離={len(grid_path) * resolution:.2f}m, '
            f'計算時間={elapsed * 1000:.1f}ms'
        )
        return path_msg

    def _run_astar(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        width: int,
        height: int,
        resolution: float,
        cost_threshold: int,
        allow_diagonal: bool,
    ) -> Optional[List[Tuple[int, int]]]:
        """Delegate to the module-level a_star_search function."""
        return a_star_search(
            list(self._map.data), width, height, start, goal,
            cost_threshold, allow_diagonal, resolution,
        )

    def _build_path_msg(
        self,
        grid_path: List[Tuple[int, int]],
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> Path:
        """Convert a list of grid cells to a nav_msgs/Path in world coordinates."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for gx, gy in grid_path:
            wx, wy = grid_to_world(gx, gy, origin_x, origin_y, resolution)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg


def main(args=None) -> None:
    """Initialise rclpy, spin the SimplePathPlanner node, and shut down cleanly."""
    rclpy.init(args=args)
    node = SimplePathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
