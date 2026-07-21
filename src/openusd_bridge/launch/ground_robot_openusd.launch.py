"""Launch the ground robot patrol and record its odometry to OpenUSD."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the ground robot plus OpenUSD recorder launch description."""
    share_dir = get_package_share_directory('openusd_bridge')
    config_file = join(share_dir, 'config', 'openusd_recording.yaml')
    output_path = LaunchConfiguration('output_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/ros2_openusd/robot_motion.usda',
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
        ),
        Node(
            package='openusd_bridge',
            executable='odom_to_usd',
            name='odom_to_usd',
            output='screen',
            parameters=[config_file, {'output_path': output_path}],
        ),
    ])
