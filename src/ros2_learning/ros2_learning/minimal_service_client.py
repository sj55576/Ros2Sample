"""ROS 2 サービスクライアントの最小構成サンプル。

このモジュールは、ROS 2 で非同期にサービスを呼び出す
最もシンプルな方法を示します。
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class MinimalServiceClient(Node):
    """2 秒ごとに 'set_flag' サービスを交互に呼び出すノード。

    非同期サービス呼び出しのパターンを学ぶためのサンプルです。
    """

    def __init__(self):
        """ノードを初期化し、サービスクライアントとタイマーを生成する。"""
        # ノード名を 'minimal_service_client' として ROS 2 に登録する
        super().__init__('minimal_service_client')

        # 次回送信する値を交互に切り替えるためのフラグ
        self._next_value = True

        # サービスクライアントを作成する
        # - サービス型: std_srvs/SetBool
        # - サービス名: 'set_flag'（サーバー側と一致させる必要がある）
        self._client = self.create_client(SetBool, 'set_flag')

        # サービスが利用可能になるまで待機する
        self.get_logger().info("'set_flag' サービスの起動を待機中...")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービスがまだ起動していません。再試行中...')

        self.get_logger().info('サービスに接続しました。')

        # 2 秒ごとにサービスを呼び出すタイマーを作成する
        self._timer = self.create_timer(2.0, self._timer_callback)

    def _timer_callback(self):
        """タイマーが発火するたびにサービスリクエストを送信する。"""
        # リクエストオブジェクトを作成して値をセットする
        request = SetBool.Request()
        request.data = self._next_value

        self.get_logger().info(
            f'サービスを呼び出します: data={request.data}'
        )

        # 非同期でサービスを呼び出す
        # call_async() はすぐに Future オブジェクトを返す
        future = self._client.call_async(request)

        # Future にコールバックを登録する
        # レスポンスが届いたときに _response_callback が呼ばれる
        future.add_done_callback(self._response_callback)

        # 次回は反対の値を送るよう切り替える
        self._next_value = not self._next_value

    def _response_callback(self, future):
        """サービスのレスポンスを受け取ったときに呼ばれるコールバック。

        future.result() でサーバーからの返答を取得できる。
        """
        response = future.result()
        self.get_logger().info(
            f'レスポンス受信: success={response.success},'
            f' message="{response.message}"'
        )


def main(args=None):
    """エントリーポイント: ノードを起動して終了まで実行する。"""
    # rclpy を初期化する（ROS 2 を使う前に必ず呼び出す）
    rclpy.init(args=args)

    node = MinimalServiceClient()

    try:
        # タイマーイベントを処理するためにスピンさせる
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時にノードを破棄してリソースを解放する
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
