"""Nav2 ウェイポイントクライアントデモ。Nav2 スタックが別途起動されている必要があります."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """nav2_waypoint_client ノードを起動する。事前に Nav2 スタックを起動すること."""
    config = os.path.join(
        get_package_share_directory('nav2_learning'),
        'config',
        'nav2_waypoint.yaml',
    )
    return LaunchDescription([
        Node(
            package='nav2_learning',
            executable='nav2_waypoint_client',
            name='nav2_waypoint_client',
            parameters=[config],
            output='screen',
        ),
    ])
