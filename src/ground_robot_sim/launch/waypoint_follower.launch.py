"""Launch one simulated robot with the closed-loop waypoint follower controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = os.path.join(
        get_package_share_directory('ground_robot_sim'),
        'config',
        'waypoint_follower.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ground_robot_sim',
            executable='ground_robot_node',
            name='ground_robot',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='ground_robot_sim',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
