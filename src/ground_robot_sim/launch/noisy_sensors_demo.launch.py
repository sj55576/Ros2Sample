"""Launch one simulated robot publishing noisy odometry and lidar for sensor-noise demos."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    share_dir = get_package_share_directory('ground_robot_sim')
    config = os.path.join(share_dir, 'config', 'noisy_sensors.yaml')

    with open(os.path.join(share_dir, 'urdf', 'ground_robot.urdf'), 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='ground_robot_sim',
            executable='ground_robot_node',
            name='ground_robot',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
