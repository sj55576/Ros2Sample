"""Launch one simulated robot with a lidar-based stop controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ground_robot_sim',
            executable='ground_robot_node',
            name='ground_robot',
            output='screen',
            parameters=[{
                'initial_x': -2.0,
                'obstacles': [0.0, 0.0, 0.45, 1.0, 1.0, 0.35],
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='ground_robot_sim',
            executable='lidar_obstacle_stop',
            name='lidar_obstacle_stop',
            output='screen',
            parameters=[{
                'stop_distance': 0.8,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
