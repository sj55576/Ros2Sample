"""占有格子地図マッピングデモ。ロボット走行→スキャン統合→OccupancyGrid配信を起動する。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """ground_robot_node・patrol・simple_occupancy_mapper・RViz を起動する。"""
    use_patrol = LaunchConfiguration('use_patrol')
    use_rviz = LaunchConfiguration('use_rviz')

    mapper_config = os.path.join(
        get_package_share_directory('nav2_learning'),
        'config',
        'occupancy_mapping.yaml',
    )
    rviz_config = os.path.join(
        get_package_share_directory('nav2_learning'),
        'rviz',
        'occupancy_mapping.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_patrol',
            default_value='true',
            description='diff_drive_patrol を起動してロボットを自律走行させるか',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='RViz2 をマッピング可視化用の設定で起動するか',
        ),
        Node(
            package='ground_robot_sim',
            executable='ground_robot_node',
            name='ground_robot',
            output='screen',
        ),
        Node(
            package='ground_robot_sim',
            executable='diff_drive_patrol',
            name='diff_drive_patrol',
            output='screen',
            condition=IfCondition(use_patrol),
        ),
        Node(
            package='nav2_learning',
            executable='simple_occupancy_mapper',
            name='simple_occupancy_mapper',
            parameters=[mapper_config],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
