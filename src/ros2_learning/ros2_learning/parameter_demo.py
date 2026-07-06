"""ROS2 パラメータの宣言・取得・動的変更を学ぶデモノード."""

from typing import List

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class ParameterDemo(Node):
    """パラメータの宣言・取得・動的変更を示すデモノード."""

    def __init__(self) -> None:
        """ノードを初期化し、パラメータとタイマーを設定する."""
        super().__init__('parameter_demo')

        # --------------------------------------------------
        # パラメータの宣言
        # declare_parameter(名前, デフォルト値) で宣言する。
        # デフォルト値を渡すと型が自動的に推論される。
        # --------------------------------------------------
        self.declare_parameter('robot_name', 'learning_bot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('enable_logging', True)

        # 初期値を読み込む
        self._robot_name = (
            self.get_parameter('robot_name').get_parameter_value().string_value
        )
        self._max_speed = (
            self.get_parameter('max_speed').get_parameter_value().double_value
        )
        self._update_rate_hz = (
            self.get_parameter(
                'update_rate_hz'
            ).get_parameter_value().double_value
        )
        self._enable_logging = (
            self.get_parameter(
                'enable_logging'
            ).get_parameter_value().bool_value
        )

        # --------------------------------------------------
        # 動的パラメータ変更コールバックの登録
        # ros2 param set でパラメータが変更されると
        # _on_parameter_change が呼ばれる。
        # --------------------------------------------------
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # パブリッシャーの作成
        self._pub = self.create_publisher(String, 'robot_info', 10)

        # タイマーの作成 (update_rate_hz の周期でパブリッシュ)
        period = 1.0 / max(self._update_rate_hz, 0.1)
        self._timer = self.create_timer(period, self._publish_info)

        self.get_logger().info(
            f'パラメータデモ起動: robot_name={self._robot_name}, '
            f'max_speed={self._max_speed}, '
            f'update_rate_hz={self._update_rate_hz}, '
            f'enable_logging={self._enable_logging}'
        )

    def _on_parameter_change(
        self, params: List[Parameter],
    ) -> SetParametersResult:
        """パラメータが変更されたときに呼ばれるコールバック."""
        # --------------------------------------------------
        # 変更前の値を保存しておき、変更後と比較してログ出力する。
        # SetParametersResult(successful=True) を返すと変更が確定する。
        # successful=False を返すと変更が拒否される。
        # --------------------------------------------------
        for param in params:
            old_val = self._get_current_value(param.name)

            if param.name == 'robot_name':
                self._robot_name = param.value
            elif param.name == 'max_speed':
                self._max_speed = param.value
            elif param.name == 'update_rate_hz':
                self._update_rate_hz = param.value
                # タイマー周期を再設定する
                self._timer.cancel()
                period = 1.0 / max(self._update_rate_hz, 0.1)
                self._timer = self.create_timer(period, self._publish_info)
            elif param.name == 'enable_logging':
                self._enable_logging = param.value

            self.get_logger().info(
                f'パラメータ変更: {param.name} '
                f'{old_val!r} -> {param.value!r}'
            )

        return SetParametersResult(successful=True)

    def _get_current_value(self, name: str) -> object:
        """指定した名前の現在のパラメータ値を返す."""
        mapping = {
            'robot_name': self._robot_name,
            'max_speed': self._max_speed,
            'update_rate_hz': self._update_rate_hz,
            'enable_logging': self._enable_logging,
        }
        return mapping.get(name, None)

    def _publish_info(self) -> None:
        """現在のパラメータ値を robot_info トピックへパブリッシュする."""
        msg = String()
        msg.data = (
            f'robot_name={self._robot_name}, '
            f'max_speed={self._max_speed:.2f}, '
            f'enable_logging={self._enable_logging}'
        )
        self._pub.publish(msg)

        # enable_logging が True の場合のみログを出力する
        if self._enable_logging:
            self.get_logger().info(f'robot_info: {msg.data}')


def main(args=None) -> None:
    """ノードのエントリーポイント."""
    rclpy.init(args=args)
    node = ParameterDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
