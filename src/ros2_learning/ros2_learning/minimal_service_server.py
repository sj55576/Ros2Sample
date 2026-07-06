"""
ROS 2 サービスサーバーの最小構成サンプル.

このモジュールは、ROS 2 でサービスリクエストを受け付けて
レスポンスを返す最もシンプルな方法を示します。
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class MinimalServiceServer(Node):
    """
    SetBool サービス 'set_flag' を提供するノード.

    クライアントからの True/False リクエストに応じて
    内部フラグを切り替えます。
    """

    def __init__(self):
        """ノードを初期化し、サービスサーバーを生成する."""
        # ノード名を 'minimal_service_server' として ROS 2 に登録する
        super().__init__('minimal_service_server')

        # 内部フラグの初期値は False
        self._flag = False

        # サービスサーバーを作成する
        # - サービス型: std_srvs/SetBool
        # - サービス名: 'set_flag'
        # - コールバック関数: _handle_set_flag
        self._srv = self.create_service(
            SetBool,
            'set_flag',
            self._handle_set_flag,
        )

        self.get_logger().info("'set_flag' サービスの待機を開始しました。")

    def _handle_set_flag(self, request, response):
        """
        サービスリクエストを処理してレスポンスを返すコールバック.

        request.data が True ならフラグを ON、False なら OFF にする。
        """
        # リクエストの data フィールドに応じてフラグを更新する
        self._flag = request.data

        # レスポンスを組み立てる
        response.success = True
        if request.data:
            response.message = 'フラグをONにしました'
        else:
            response.message = 'フラグをOFFにしました'

        self.get_logger().info(
            f'サービス呼び出し: data={request.data} -> {response.message}'
        )

        return response


def main(args=None):
    """エントリーポイント: ノードを起動して終了まで実行する."""
    # rclpy を初期化する（ROS 2 を使う前に必ず呼び出す）
    rclpy.init(args=args)

    node = MinimalServiceServer()

    try:
        # リクエストが届くたびにコールバックが呼ばれるよう待機する
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時にノードを破棄してリソースを解放する
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
