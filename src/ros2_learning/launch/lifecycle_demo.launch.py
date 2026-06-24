"""ライフサイクルノードを起動するデモ。"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    """ライフサイクルデモの LaunchDescription を生成する。"""
    return LaunchDescription([
        LifecycleNode(
            package='ros2_learning',
            executable='lifecycle_demo',
            name='lifecycle_demo',
            namespace='',
            output='screen',
            parameters=[{
                'publish_rate_hz': 1.0,
                'message_prefix': 'ライフサイクル',
            }],
        ),
    ])
