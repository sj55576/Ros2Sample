"""Launch the manipulator simulator with IK target commander."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build a launch description for the IK demo."""
    share_dir = get_package_share_directory('manipulator_sim')
    default_config = join(share_dir, 'config', 'ik_demo.yaml')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='manipulator_sim',
            executable='manipulator_simulator',
            name='manipulator_simulator',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='manipulator_sim',
            executable='ik_target_commander',
            name='ik_target_commander',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
