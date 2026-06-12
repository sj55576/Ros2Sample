"""Launch the simulated quadrotor with a tiny altitude-hold controller."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('drone_sim')
    default_config = join(share_dir, 'config', 'altitude_hold.yaml')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_altitude_m = LaunchConfiguration('target_altitude_m')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('target_altitude_m', default_value='2.0'),
        Node(
            package='drone_sim',
            executable='sim_drone',
            name='sim_drone',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='drone_sim',
            executable='altitude_hold',
            name='altitude_hold',
            output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'target_altitude_m': target_altitude_m,
            }],
        ),
    ])
