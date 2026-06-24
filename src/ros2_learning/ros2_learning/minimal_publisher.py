"""ROS 2 パブリッシャーの最小構成サンプル。

このモジュールは、ROS 2 でトピックにメッセージを送信する
最もシンプルな方法を示します。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """'chatter' トピックへ文字列メッセージを定期送信するノード。

    ROS 2 パブリッシャーの基本的な使い方を学ぶためのサンプルです。
    """

    def __init__(self):
        """ノードを初期化し、パブリッシャーとタイマーを生成する。"""
        # ノード名を 'minimal_publisher' として ROS 2 に登録する
        super().__init__('minimal_publisher')

        # パラメータを宣言する: publish_rate_hz（デフォルト 1.0 Hz）
        # パラメータを使うと、起動時に外部から値を変更できる
        self.declare_parameter('publish_rate_hz', 1.0)
        rate = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value

        # パブリッシャーを作成する
        # - 型: std_msgs/String
        # - トピック名: 'chatter'
        # - QoS キューサイズ: 10
        self._publisher = self.create_publisher(String, 'chatter', 10)

        # 送信したメッセージの通し番号
        self._count = 0

        # タイマーを作成して定期的に _timer_callback を呼び出す
        # タイマー周期は 1/rate 秒
        timer_period = 1.0 / rate
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'パブリッシャーを起動しました。送信レート: {rate} Hz'
        )

    def _timer_callback(self):
        """タイマーが発火するたびにメッセージを送信する。"""
        # 送信するメッセージを組み立てる
        msg = String()
        msg.data = (
            f'こんにちは ROS 2! メッセージ番号: {self._count}'
        )

        # トピックへ送信する
        self._publisher.publish(msg)

        # ログに記録して動作を確認できるようにする
        self.get_logger().info(f'送信: "{msg.data}"')

        self._count += 1


def main(args=None):
    """エントリーポイント: ノードを起動して終了まで実行する。"""
    # rclpy を初期化する（ROS 2 を使う前に必ず呼び出す）
    rclpy.init(args=args)

    node = MinimalPublisher()

    try:
        # ノードが停止されるまでイベントループを回し続ける
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時にノードを破棄してリソースを解放する
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
