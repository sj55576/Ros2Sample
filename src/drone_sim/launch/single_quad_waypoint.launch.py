"""Launch one simulated quadrotor and a waypoint commander."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('drone_sim')
    default_config = join(share_dir, 'config', 'single_quad.yaml')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='drone_sim',
            executable='sim_drone',
            name='sim_drone',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='drone_sim',
            executable='waypoint_commander',
            name='waypoint_commander',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
