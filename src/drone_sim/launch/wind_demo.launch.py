"""Launch a wind disturbance demo with geofence monitoring and telemetry logging."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('drone_sim')
    config_file = join(share_dir, 'config', 'wind_demo.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='drone_sim',
            executable='sim_drone',
            name='sim_drone',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='drone_sim',
            executable='waypoint_commander',
            name='waypoint_commander',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='drone_sim',
            executable='wind_disturbance',
            name='wind_disturbance',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='drone_sim',
            executable='geofence_monitor',
            name='geofence_monitor',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='drone_sim',
            executable='telemetry_logger',
            name='telemetry_logger',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
    ])
