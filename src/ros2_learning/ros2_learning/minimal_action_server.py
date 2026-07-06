"""シンプルなウェイポイント巡回アクションサーバーの実装例."""

import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sample_interfaces.action import NavigateWaypoints


class MinimalActionServer(Node):
    """NavigateWaypoints アクションを処理するサーバーノード."""

    def __init__(self) -> None:
        """ノードを初期化しアクションサーバーを作成する."""
        super().__init__('minimal_action_server')

        # シミュレーション用の現在位置
        self._x = 0.0
        self._y = 0.0

        # 移動速度 (m/s) — パラメータで変更可能
        self.declare_parameter('speed', 0.5)
        # 制御ループ周期 (Hz)
        self.declare_parameter('control_rate_hz', 10.0)

        # アクションサーバーの作成
        # 3つのコールバックを登録する:
        #   goal_callback:    ゴール受付の判断
        #   cancel_callback:  キャンセル要求の判断
        #   execute_callback: 実際の処理
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

        self.get_logger().info(
            'アクションサーバー起動: navigate_waypoints'
        )

    def _on_goal(
        self, goal_request: NavigateWaypoints.Goal,
    ) -> GoalResponse:
        """ゴール要求を受け付けるか判断する."""
        n = len(goal_request.waypoints)
        if n == 0:
            self.get_logger().warn('空のウェイポイント — 拒否')
            return GoalResponse.REJECT
        self.get_logger().info(f'ゴール受付: {n} 個のウェイポイント')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        """キャンセル要求を常に受け入れる."""
        self.get_logger().info('キャンセル要求を受理')
        return CancelResponse.ACCEPT

    def _execute(
        self, goal_handle,
    ) -> NavigateWaypoints.Result:
        """ウェイポイントを順番に巡回する."""
        speed = float(self.get_parameter('speed').value)
        rate_hz = max(
            1.0, float(self.get_parameter('control_rate_hz').value),
        )
        dt = 1.0 / rate_hz

        waypoints = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in goal_handle.request.waypoints
        ]
        loop = bool(goal_handle.request.loop)
        tolerance = max(
            0.05, float(goal_handle.request.tolerance_m or 0.2),
        )

        idx = 0
        completed = 0
        feedback = NavigateWaypoints.Feedback()

        self.get_logger().info(
            f'実行開始: {len(waypoints)} WP, '
            f'loop={loop}, tol={tolerance:.2f} m'
        )

        while rclpy.ok():
            # キャンセル確認
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(
                    False, completed, 'キャンセルされました',
                )

            if idx >= len(waypoints):
                break

            tx, ty = waypoints[idx]
            dx = tx - self._x
            dy = ty - self._y
            dist = math.hypot(dx, dy)

            # フィードバック送信
            feedback.current_index = idx
            feedback.total_waypoints = len(waypoints)
            feedback.distance_to_current = dist
            feedback.current_position = Point(
                x=self._x, y=self._y, z=0.0,
            )
            goal_handle.publish_feedback(feedback)

            if dist <= tolerance:
                completed += 1
                self.get_logger().info(
                    f'WP {idx} 到達: '
                    f'({tx:.2f}, {ty:.2f})'
                )
                nxt = idx + 1
                if nxt < len(waypoints):
                    idx = nxt
                elif loop:
                    idx = 0
                else:
                    idx = len(waypoints)
            else:
                # 目標方向へ等速移動
                step = min(speed * dt, dist)
                self._x += dx / dist * step
                self._y += dy / dist * step

            time.sleep(dt)

        goal_handle.succeed()
        return self._make_result(
            True, completed,
            f'{completed} 個のウェイポイントを完了',
        )

    def _make_result(
        self, success: bool, count: int, msg: str,
    ) -> NavigateWaypoints.Result:
        """結果メッセージを組み立てる."""
        result = NavigateWaypoints.Result()
        result.success = success
        result.waypoints_completed = count
        result.message = msg
        self.get_logger().info(f'結果: {msg}')
        return result


def main(args=None) -> None:
    """エントリーポイント."""
    rclpy.init(args=args)
    node = MinimalActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
