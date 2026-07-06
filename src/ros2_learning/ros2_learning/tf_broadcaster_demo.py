"""TF ブロードキャスターのデモ: 動的変換と静的変換の両方を配信する."""

# ============================================================
# TF ブロードキャスター デモ
#
# 【TF とは?】
#   TF (Transform Framework) は ROS2 において、異なる座標フレーム間の
#   変換 (位置・姿勢) を管理するライブラリです。
#   例: ロボットの胴体フレーム → カメラフレームの変換を記録・参照できる。
#
# 【動的変換 (TransformBroadcaster)】
#   時間とともに変化する変換を配信する。
#   例: ロボットが移動するときの world → base_link の変換。
#
# 【静的変換 (StaticTransformBroadcaster)】
#   変化しない固定の変換を配信する。
#   例: ロボット本体 → センサーの取り付け位置。
#   一度だけ配信すれば十分で、ラッチされたトピックで配信される。
#
# 【フレーム階層】
#   world
#   └─ learning_robot  (動的: 円軌道を移動)
#      └─ sensor_frame (静的: Z軸方向に 0.1m オフセット)
# ============================================================

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TfBroadcasterDemo(Node):
    """円軌道の動的変換と固定の静的変換を配信するデモノード."""

    def __init__(self) -> None:
        """ノードを初期化し、TFブロードキャスターとタイマーを設定する."""
        super().__init__('tf_broadcaster_demo')

        # パラメータの宣言
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'learning_robot')
        self.declare_parameter('orbit_radius', 2.0)
        self.declare_parameter('orbit_speed', 0.5)

        self._parent_frame = (
            self.get_parameter(
                'parent_frame'
            ).get_parameter_value().string_value
        )
        self._child_frame = (
            self.get_parameter(
                'child_frame'
            ).get_parameter_value().string_value
        )
        self._orbit_radius = (
            self.get_parameter(
                'orbit_radius'
            ).get_parameter_value().double_value
        )
        self._orbit_speed = (
            self.get_parameter(
                'orbit_speed'
            ).get_parameter_value().double_value
        )

        # --------------------------------------------------
        # 動的変換ブロードキャスターの作成
        # タイマーコールバックの中で繰り返し変換を配信する。
        # --------------------------------------------------
        self._broadcaster = TransformBroadcaster(self)

        # --------------------------------------------------
        # 静的変換ブロードキャスターの作成
        # センサーフレームはロボット本体に固定されているため
        # 一度だけ配信すれば十分である。
        # --------------------------------------------------
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transform()

        # 30 Hz で動的変換を配信するタイマー
        self._timer = self.create_timer(1.0 / 30.0, self._publish_transform)
        self._angle = 0.0  # 現在の軌道角度 [rad]

        self.get_logger().info(
            f'TFブロードキャスター起動: '
            f'{self._parent_frame} -> {self._child_frame}, '
            f'半径={self._orbit_radius}m, '
            f'速度={self._orbit_speed}rad/s'
        )

    def _publish_static_transform(self) -> None:
        """sensor_frame の静的変換を一度だけ配信する."""
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 静的変換の親フレームは child_frame (= learning_robot)
        msg.header.frame_id = self._child_frame
        msg.child_frame_id = 'sensor_frame'

        # センサーはロボット本体の Z 軸方向に 0.1m 取り付けられている
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.1

        # 回転なし (単位クォータニオン)
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0

        self._static_broadcaster.sendTransform(msg)
        self.get_logger().info(
            f'静的変換配信: {self._child_frame} -> sensor_frame '
            f'(Z オフセット 0.1m)'
        )

    def _publish_transform(self) -> None:
        """円軌道の動的変換を 30 Hz で配信する."""
        # タイムステップごとに角度を更新する
        dt = 1.0 / 30.0
        self._angle += self._orbit_speed * dt

        # XY 平面上の円軌道座標を計算する
        x = self._orbit_radius * math.cos(self._angle)
        y = self._orbit_radius * math.sin(self._angle)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._parent_frame
        msg.child_frame_id = self._child_frame

        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = 0.0

        # ロボットは進行方向を向くようにヨー回転させる
        # ヨー角 θ の場合: qz = sin(θ/2), qw = cos(θ/2)
        yaw = self._angle + math.pi / 2.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)

        self._broadcaster.sendTransform(msg)


def main(args=None) -> None:
    """ノードのエントリーポイント."""
    rclpy.init(args=args)
    node = TfBroadcasterDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
