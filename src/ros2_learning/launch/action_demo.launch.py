"""アクションサーバーとクライアントを同時に起動するデモ."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """アクションデモの LaunchDescription を生成する."""
    return LaunchDescription([
        Node(
            package='ros2_learning',
            executable='minimal_action_server',
            name='minimal_action_server',
            output='screen',
        ),
        Node(
            package='ros2_learning',
            executable='minimal_action_client',
            name='minimal_action_client',
            output='screen',
        ),
    ])
