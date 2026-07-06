"""シンプルなウェイポイント巡回アクションクライアントの実装例."""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sample_interfaces.action import NavigateWaypoints


class MinimalActionClient(Node):
    """NavigateWaypoints アクションにゴールを送信するクライアント."""

    def __init__(self) -> None:
        """ノードを初期化しアクションクライアントを作成する."""
        super().__init__('minimal_action_client')

        # アクションクライアントの作成
        self._client = ActionClient(
            self, NavigateWaypoints, 'navigate_waypoints',
        )

        # 3秒後にゴールを送信する（サーバー起動待ち）
        self.create_timer(3.0, self._send_goal_once)
        self._goal_sent = False

        self.get_logger().info(
            'アクションクライアント起動: '
            '3秒後にゴールを送信します'
        )

    def _send_goal_once(self) -> None:
        """一度だけゴールを送信する."""
        if self._goal_sent:
            return
        self._goal_sent = True

        # サーバー接続を待つ
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'アクションサーバーが見つかりません'
            )
            return

        # ウェイポイント列を作成
        # 正方形の4頂点を巡回する
        coords = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]
        waypoints = []
        for x, y in coords:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            waypoints.append(ps)

        # ゴールメッセージの組み立て
        goal = NavigateWaypoints.Goal()
        goal.waypoints = waypoints
        goal.loop = False
        goal.tolerance_m = 0.2

        self.get_logger().info(
            f'{len(waypoints)} 個のウェイポイントを送信'
        )

        # ゴール送信（非同期）
        # feedback_callback でフィードバックを受け取る
        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        """サーバーがゴールを受け付けたかを確認する."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('ゴールが拒否されました')
            return

        self.get_logger().info('ゴールが受け付けられました')

        # 結果を非同期で取得
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        """フィードバックを受信して表示する."""
        fb = feedback_msg.feedback
        pos = fb.current_position
        self.get_logger().info(
            f'フィードバック: WP {fb.current_index}/'
            f'{fb.total_waypoints} '
            f'残り {fb.distance_to_current:.2f} m '
            f'位置 ({pos.x:.2f}, {pos.y:.2f})'
        )

    def _on_result(self, future) -> None:
        """最終結果を受信して表示する."""
        result = future.result().result
        status = '成功' if result.success else '失敗'
        self.get_logger().info(
            f'結果: {status} — '
            f'{result.waypoints_completed} WP完了 — '
            f'{result.message}'
        )


def main(args=None) -> None:
    """エントリーポイント."""
    rclpy.init(args=args)
    node = MinimalActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
