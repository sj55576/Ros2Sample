"""A* path planner on OccupancyGrid for learning purposes."""

import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_learning.map_utils import (
    a_star_search,
    grid_to_world,
    is_valid_cell,
    world_to_grid,
)
from nav2_learning.path_utils import (
    bresenham_line,
    is_path_blocked,
    shortcut_path,
    smooth_path_moving_average,
)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger


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
        self.declare_parameter('shortcut_enabled', True)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('replan_on_map_change', True)
        self.declare_parameter('use_odom_start', False)

        self._map: Optional[OccupancyGrid] = None
        self._odom_position: Optional[Tuple[float, float]] = None
        self._last_path_cells: List[Tuple[int, int]] = []

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, latched_qos)
        self._plan_pub = self.create_publisher(Path, '/plan', 10)
        self._plan_raw_pub = self.create_publisher(Path, '/plan_raw', 10)
        self.create_service(Trigger, '~/plan_path', self._plan_service_callback)

        if bool(self.get_parameter('use_odom_start').value):
            self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        replan_rate = float(self.get_parameter('replan_rate').value)
        if replan_rate > 0.0:
            self.create_timer(1.0 / replan_rate, self._plan_and_publish)

        self.get_logger().info('パスプランナー起動: マップを待っています...')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Store the received map and trigger initial planning or replanning on change."""
        is_first_map = self._map is None
        self._map = msg
        self.get_logger().info(
            f'マップ受信: {msg.info.width}x{msg.info.height}セル'
        )

        replan_rate = float(self.get_parameter('replan_rate').value)
        if replan_rate > 0.0:
            return

        if is_first_map:
            self._plan_and_publish()
        elif self._is_current_path_blocked(msg):
            self.get_logger().warn('経路上に障害物を検知、再計画します')
            self._plan_and_publish()

    def _is_current_path_blocked(self, msg: OccupancyGrid) -> bool:
        """Check whether the updated map now blocks the previously published path."""
        replan_on_map_change = bool(self.get_parameter('replan_on_map_change').value)
        if not replan_on_map_change or not self._last_path_cells:
            return False
        cost_threshold = int(self.get_parameter('cost_threshold').value)
        return is_path_blocked(
            list(msg.data), msg.info.width, msg.info.height,
            self._last_path_cells, cost_threshold,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache the latest odometry position for use as the planning start."""
        self._odom_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

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
        """Run A*, shortcut/smooth the path, and publish /plan and /plan_raw."""
        if self._map is None:
            return None

        start_x, start_y = self._resolve_start()
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

        raw_msg = self._build_path_msg(grid_path, ox, oy, resolution)
        self._plan_raw_pub.publish(raw_msg)

        path_msg = self._finalize_path(grid_path, cost_threshold, ox, oy, resolution)
        self._plan_pub.publish(path_msg)
        self.get_logger().info(
            f'パス生成完了: {len(grid_path)}セル, '
            f'距離={len(grid_path) * resolution:.2f}m, '
            f'計算時間={elapsed * 1000:.1f}ms'
        )
        return path_msg

    def _resolve_start(self) -> Tuple[float, float]:
        """Determine the planning start position, preferring odometry if configured."""
        if bool(self.get_parameter('use_odom_start').value):
            if self._odom_position is not None:
                return self._odom_position
            self.get_logger().warn(
                'use_odom_startが有効ですがオドメトリ未受信のため、start_x/start_yを使用します'
            )
        start_x = float(self.get_parameter('start_x').value)
        start_y = float(self.get_parameter('start_y').value)
        return start_x, start_y

    def _finalize_path(
        self,
        grid_path: List[Tuple[int, int]],
        cost_threshold: int,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> Path:
        """Apply shortcutting/smoothing to the raw A* path and build the publishable message."""
        if bool(self.get_parameter('shortcut_enabled').value):
            info = self._map.info
            cells = shortcut_path(
                list(self._map.data), info.width, info.height, grid_path, cost_threshold
            )
        else:
            cells = list(grid_path)

        self._last_path_cells = self._interpolate_cells(cells)

        points = [grid_to_world(gx, gy, origin_x, origin_y, resolution) for gx, gy in cells]
        smoothing_window = int(self.get_parameter('smoothing_window').value)
        if smoothing_window > 0:
            points = smooth_path_moving_average(points, smoothing_window)

        return self._build_path_msg_from_points(points)

    def _interpolate_cells(
        self,
        cells: List[Tuple[int, int]],
    ) -> List[Tuple[int, int]]:
        """Interpolate every consecutive waypoint pair via Bresenham into all traversed cells."""
        if not cells:
            return []
        interpolated: List[Tuple[int, int]] = [cells[0]]
        for i in range(1, len(cells)):
            segment = bresenham_line(cells[i - 1][0], cells[i - 1][1], cells[i][0], cells[i][1])
            interpolated.extend(segment[1:])
        return interpolated

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

    def _build_path_msg_from_points(self, points: List[Tuple[float, float]]) -> Path:
        """Build a nav_msgs/Path directly from a list of world-coordinate points."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wx, wy in points:
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
