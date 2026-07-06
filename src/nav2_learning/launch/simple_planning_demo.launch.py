"""経路計画と追従のデモ。マップ生成→A*計画→Pure Pursuit追従の全パイプラインを起動."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ground_robot_node・マップ・プランナー・フォロワーを全て起動する."""
    nav2_config = os.path.join(
        get_package_share_directory('nav2_learning'),
        'config',
        'simple_planning.yaml',
    )
    map_config = os.path.join(
        get_package_share_directory('nav2_learning'),
        'config',
        'map_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='ground_robot_sim',
            executable='ground_robot_node',
            name='ground_robot',
            output='screen',
        ),
        Node(
            package='nav2_learning',
            executable='simple_map_publisher',
            name='simple_map_publisher',
            parameters=[map_config],
            output='screen',
        ),
        Node(
            package='nav2_learning',
            executable='simple_path_planner',
            name='simple_path_planner',
            parameters=[nav2_config],
            output='screen',
        ),
        Node(
            package='nav2_learning',
            executable='simple_path_follower',
            name='simple_path_follower',
            parameters=[nav2_config],
            output='screen',
        ),
    ])
