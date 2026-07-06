"""tf_broadcaster_demo と tf_listener_demo を同時に起動するランチファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """TF デモノード群のランチディスクリプションを生成する."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='シミュレーション時刻を使用するかどうか',
        ),
        # ブロードキャスター: 円軌道の動的変換と静的変換を配信する
        Node(
            package='ros2_learning',
            executable='tf_broadcaster_demo',
            name='tf_broadcaster_demo',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # リスナー: world -> sensor_frame の変換を取得してログ出力する
        Node(
            package='ros2_learning',
            executable='tf_listener_demo',
            name='tf_listener_demo',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
