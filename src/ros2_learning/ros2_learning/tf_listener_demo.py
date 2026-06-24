"""TF リスナーのデモ: フレーム間の変換を取得してログ出力する。"""

# ============================================================
# TF リスナー デモ
#
# 【変換の取得方法】
#   Buffer.lookup_transform(target_frame, source_frame, time) を使う。
#   「source_frame から見た target_frame の位置・姿勢」が返る。
#   time に rclpy.time.Time() を渡すと最新の変換を取得できる。
#
# 【よくある例外と対処法】
#   LookupException     : 指定したフレームが存在しない。
#                         → ブロードキャスター側が起動しているか確認する。
#   ConnectivityException: フレーム間にパスが存在しない。
#                          → TF ツリーが正しく構築されているか確認する。
#   ExtrapolationException: 要求した時刻のデータがバッファにない。
#                           → タイムアウトを設定するか、最新時刻を使う。
#
# 【実用例】
#   センサーデータを取得するとき、センサーフレームの座標を
#   ロボット本体やワールド座標系に変換するために使う。
#   例: カメラで検出した物体位置を world 座標に変換する。
# ============================================================

import math

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfListenerDemo(Node):
    """フレーム間の変換を定期的に取得してログ出力するデモノード。"""

    def __init__(self) -> None:
        """ノードを初期化し、TFバッファとリスナーを設定する。"""
        super().__init__('tf_listener_demo')

        # パラメータの宣言
        self.declare_parameter('target_frame', 'sensor_frame')
        self.declare_parameter('source_frame', 'world')

        self._target_frame = (
            self.get_parameter(
                'target_frame'
            ).get_parameter_value().string_value
        )
        self._source_frame = (
            self.get_parameter(
                'source_frame'
            ).get_parameter_value().string_value
        )

        # --------------------------------------------------
        # TF バッファとリスナーの作成
        # Buffer は変換データをキャッシュする。
        # TransformListener はトピックを購読して Buffer を更新する。
        # この2つはセットで使う。
        # --------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 2 Hz で変換を取得するタイマー
        self._timer = self.create_timer(0.5, self._lookup_transform)

        self.get_logger().info(
            f'TFリスナー起動: '
            f'{self._source_frame} -> {self._target_frame} の変換を監視'
        )

    def _lookup_transform(self) -> None:
        """source_frame から target_frame への変換を取得してログ出力する。"""
        try:
            # --------------------------------------------------
            # lookup_transform(target, source, time, timeout)
            # 「source から見た target の変換」を取得する。
            # rclpy.time.Time() は最新の変換を意味する。
            # timeout を設定すると、データが届くまで待機する。
            # --------------------------------------------------
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # 原点からのユークリッド距離を計算する
            distance = math.sqrt(tx ** 2 + ty ** 2 + tz ** 2)

            self.get_logger().info(
                f'変換取得成功: '
                f'{self._source_frame} -> {self._target_frame} | '
                f'位置: x={tx:.3f}, y={ty:.3f}, z={tz:.3f} [m] | '
                f'原点からの距離: {distance:.3f} m'
            )

        except tf2_ros.LookupException as e:
            # フレームが存在しない場合
            # tf_broadcaster_demo が起動しているか確認してください
            self.get_logger().warn(
                f'フレームが見つかりません: {e}',
                throttle_duration_sec=2.0,
            )

        except tf2_ros.ConnectivityException as e:
            # フレーム間のパスが存在しない場合
            # TF ツリーの接続が切れている可能性があります
            self.get_logger().warn(
                f'フレーム間の接続がありません: {e}',
                throttle_duration_sec=2.0,
            )

        except tf2_ros.ExtrapolationException as e:
            # 要求した時刻のデータがバッファにない場合
            # 起動直後は変換データが届いていないため発生しやすい
            self.get_logger().warn(
                f'変換データの時刻が範囲外です: {e}',
                throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    """ノードのエントリーポイント。"""
    rclpy.init(args=args)
    node = TfListenerDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
