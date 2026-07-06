"""Action client for Nav2 NavigateToPose - demonstrates Nav2 integration."""

import math
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False


def _parse_waypoints_xy(raw: List[float]) -> List[Tuple[float, float]]:
    """Parse a flat [x1, y1, x2, y2, ...] list into (x, y) tuples."""
    if len(raw) % 2 != 0:
        raise ValueError('waypoints parameter length must be even (x, y pairs)')
    values = [float(v) for v in raw]
    return [(values[i], values[i + 1]) for i in range(0, len(values), 2)]


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) quaternion for a planar yaw angle."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Nav2WaypointClient(Node):
    """Send NavigateToPose goals to Nav2 - single goal or sequential waypoints."""

    def __init__(self) -> None:
        super().__init__('nav2_waypoint_client')
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('use_waypoints', False)
        self.declare_parameter('waypoints', [1.0, 0.0, 2.0, 1.0, 2.0, 2.0])

        if not HAS_NAV2:
            self.get_logger().error(
                'nav2_msgs がインストールされていません。\n'
                'インストール方法: sudo apt install ros-jazzy-nav2-msgs\n'
                'またはワークスペースに nav2_msgs を追加してください。'
            )
            return

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._waypoint_index: int = 0
        self._use_waypoints = bool(self.get_parameter('use_waypoints').value)

        if self._use_waypoints:
            raw = list(self.get_parameter('waypoints').value)
            self._waypoints = _parse_waypoints_xy(raw)
            self.get_logger().info(
                f'ウェイポイントモード: {len(self._waypoints)}点'
            )
        else:
            self._waypoints = []

        # Wait for the action server in a one-shot timer so __init__ returns quickly.
        self._connect_timer = self.create_timer(0.5, self._try_connect)

    def _try_connect(self) -> None:
        """Wait for the Nav2 action server and send the first goal once connected."""
        self._connect_timer.cancel()
        self.get_logger().info('Nav2 アクションサーバーに接続中...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Nav2 アクションサーバーが見つかりません。Nav2 が起動しているか確認してください。'
            )
            return
        self.get_logger().info('Nav2 アクションサーバーに接続しました')
        self._send_next_goal()

    def _send_next_goal(self) -> None:
        """Send the current goal or next waypoint to Nav2."""
        if self._use_waypoints:
            if self._waypoint_index >= len(self._waypoints):
                self.get_logger().info('全ウェイポイント完了')
                return
            wx, wy = self._waypoints[self._waypoint_index]
            wyaw = 0.0
            self.get_logger().info(
                f'ウェイポイント {self._waypoint_index + 1}/{len(self._waypoints)}: '
                f'({wx:.2f}, {wy:.2f})'
            )
        else:
            wx = float(self.get_parameter('goal_x').value)
            wy = float(self.get_parameter('goal_y').value)
            wyaw = float(self.get_parameter('goal_yaw').value)
            self.get_logger().info(f'ゴール送信: ({wx:.2f}, {wy:.2f}, yaw={wyaw:.2f}rad)')

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        qx, qy, qz, qw = _yaw_to_quaternion(wyaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        """Handle the goal acceptance/rejection response from Nav2."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('ゴールが拒否されました')
            return
        self.get_logger().info('ゴール受理 - ナビゲーション開始')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        """Log navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        dist = getattr(feedback, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().info(f'残り距離: {dist:.2f}m', throttle_duration_sec=2.0)

    def _result_callback(self, future) -> None:
        """Handle the navigation result and advance to the next waypoint if applicable."""
        result = future.result()
        status = result.status

        # action_msgs/GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6
        if status == 4:
            self.get_logger().info('ナビゲーション成功')
            if self._use_waypoints:
                self._waypoint_index += 1
                self._send_next_goal()
        elif status == 5:
            self.get_logger().warn('ナビゲーションがキャンセルされました')
        else:
            self.get_logger().error(f'ナビゲーション失敗 (status={status})')


def main(args=None) -> None:
    """Initialise rclpy, spin the Nav2WaypointClient node, and shut down cleanly."""
    if not HAS_NAV2:
        print(
            '[nav2_waypoint_client] nav2_msgs が見つかりません。\n'
            'インストール: sudo apt install ros-jazzy-nav2-msgs\n'
            'その後 colcon build --packages-select nav2_learning を実行してください。'
        )
        return
    rclpy.init(args=args)
    node = Nav2WaypointClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
