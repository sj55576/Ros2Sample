"""マップ生成デモの Launch ファイル。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """simple_map_publisher と costmap_monitor を起動する。"""
    config = os.path.join(
        get_package_share_directory('nav2_learning'),
        'config',
        'map_params.yaml',
    )
    return LaunchDescription([
        Node(
            package='nav2_learning',
            executable='simple_map_publisher',
            name='simple_map_publisher',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='nav2_learning',
            executable='costmap_monitor',
            name='costmap_monitor',
            output='screen',
        ),
    ])
