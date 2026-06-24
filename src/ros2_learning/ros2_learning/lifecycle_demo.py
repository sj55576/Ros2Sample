"""ライフサイクルノードの状態遷移を学ぶデモ。"""

import rclpy
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    TransitionCallbackReturn,
)
from rclpy.lifecycle.node import LifecyclePublisher
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class LifecycleDemo(LifecycleNode):
    """状態遷移を体験するためのライフサイクルノード。

    状態遷移の流れ:
      Unconfigured --configure--> Inactive
      Inactive     --activate-->  Active
      Active       --deactivate-> Inactive
      Inactive     --cleanup-->   Unconfigured
    """

    def __init__(self) -> None:
        """ノードを未設定状態で初期化する。"""
        super().__init__('lifecycle_demo')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('message_prefix', 'ライフサイクル')

        self._pub = None
        self._timer = None
        self._count = 0

        self.get_logger().info(
            '作成完了 — 状態: Unconfigured'
        )
        self.get_logger().info(
            '次のコマンドで状態を遷移してください:'
        )
        self.get_logger().info(
            '  ros2 lifecycle set /lifecycle_demo configure'
        )

    # --- 状態遷移コールバック ---

    def on_configure(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """リソースを確保しパブリッシャーを作成する。"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_lifecycle_publisher(
            String, 'lifecycle_output', qos,
        )
        self._count = 0

        self.get_logger().info(
            'on_configure: パブリッシャー作成完了'
        )
        self.get_logger().info(
            '  → ros2 lifecycle set /lifecycle_demo activate'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """タイマーを開始してメッセージ配信を始める。"""
        rate = float(
            self.get_parameter('publish_rate_hz').value,
        )
        period = 1.0 / max(rate, 0.1)
        self._timer = self.create_timer(
            period, self._publish,
        )

        self.get_logger().info(
            f'on_activate: {rate:.1f} Hz で配信開始'
        )
        self.get_logger().info(
            '  → ros2 lifecycle set '
            '/lifecycle_demo deactivate'
        )
        return super().on_activate(state)

    def on_deactivate(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """タイマーを停止して配信を一時停止する。"""
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        self.get_logger().info(
            f'on_deactivate: 配信停止 '
            f'(合計 {self._count} メッセージ送信済み)'
        )
        self.get_logger().info(
            '  → ros2 lifecycle set /lifecycle_demo cleanup'
        )
        return super().on_deactivate(state)

    def on_cleanup(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """リソースを解放して初期状態に戻す。"""
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        self._count = 0

        self.get_logger().info(
            'on_cleanup: 初期状態に戻りました'
        )
        self.get_logger().info(
            '  → ros2 lifecycle set '
            '/lifecycle_demo configure'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(
        self, state: LifecycleState,
    ) -> TransitionCallbackReturn:
        """どの状態からでもシャットダウンを処理する。"""
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info(
            f'on_shutdown: 終了 '
            f'(前の状態: {state.label})'
        )
        return TransitionCallbackReturn.SUCCESS

    # --- 内部処理 ---

    def _publish(self) -> None:
        """アクティブ状態でのみメッセージを配信する。"""
        if self._pub is None:
            return
        if not self._pub.is_activated:
            return

        prefix = str(
            self.get_parameter('message_prefix').value,
        )
        msg = String()
        msg.data = f'{prefix} #{self._count}'
        self._pub.publish(msg)
        self.get_logger().info(f'配信: {msg.data}')
        self._count += 1


def main(args=None) -> None:
    """エントリーポイント。"""
    rclpy.init(args=args)
    node = LifecycleDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
