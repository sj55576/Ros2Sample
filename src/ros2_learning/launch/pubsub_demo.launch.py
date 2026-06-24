"""パブリッシャーとサブスクライバーを同時に起動するランチファイル。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """minimal_publisher と minimal_subscriber を起動する。"""
    # パブリッシャーノードの定義
    publisher_node = Node(
        package='ros2_learning',
        executable='minimal_publisher',
        name='minimal_publisher',
        output='screen',
    )

    # サブスクライバーノードの定義
    subscriber_node = Node(
        package='ros2_learning',
        executable='minimal_subscriber',
        name='minimal_subscriber',
        output='screen',
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
    ])
