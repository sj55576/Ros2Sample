"""ROS 2 サブスクライバーの最小構成サンプル。

このモジュールは、ROS 2 でトピックからメッセージを受信する
最もシンプルな方法を示します。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """'chatter' トピックを購読して受信メッセージをログ出力するノード。

    ROS 2 サブスクライバーの基本的な使い方を学ぶためのサンプルです。
    """

    def __init__(self):
        """ノードを初期化し、サブスクライバーを生成する。"""
        # ノード名を 'minimal_subscriber' として ROS 2 に登録する
        super().__init__('minimal_subscriber')

        # サブスクライバーを作成する
        # - 型: std_msgs/String
        # - トピック名: 'chatter'（パブリッシャーと一致させる必要がある）
        # - QoS キューサイズ: 10
        # - コールバック関数: _listener_callback
        self._subscription = self.create_subscription(
            String,
            'chatter',
            self._listener_callback,
            10,
        )

        self.get_logger().info(
            "'chatter' トピックの受信を開始しました。"
        )

    def _listener_callback(self, msg):
        """メッセージを受信したときに呼ばれるコールバック関数。

        サブスクライバーは新しいメッセージが届くたびにこの関数を呼び出す。
        引数 msg には受信したメッセージオブジェクトが入っている。
        """
        # 受信したメッセージの内容をログに出力する
        self.get_logger().info(f'受信: "{msg.data}"')


def main(args=None):
    """エントリーポイント: ノードを起動して終了まで実行する。"""
    # rclpy を初期化する（ROS 2 を使う前に必ず呼び出す）
    rclpy.init(args=args)

    node = MinimalSubscriber()

    try:
        # メッセージが届くたびにコールバックが呼ばれるよう待機する
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時にノードを破棄してリソースを解放する
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
