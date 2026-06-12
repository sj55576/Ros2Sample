"""Launch several independent simulated robots in ROS namespaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


ROBOTS = [
    ('robot1', -1.5, -1.0, 0.0),
    ('robot2', 1.5, -1.0, 3.14),
    ('robot3', 0.0, 1.3, -1.57),
]


def robot_group(name, x, y, yaw, use_sim_time):
    """Create namespaced simulator and patrol nodes for one robot."""
    return GroupAction([
        PushRosNamespace(name),
        Node(
            package='ground_robot_sim',
            executable='ground_robot_node',
            name='ground_robot',
            output='screen',
            parameters=[{
                'robot_name': name,
                'frame_prefix': name,
                'initial_x': x,
                'initial_y': y,
                'initial_yaw': yaw,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='ground_robot_sim',
            executable='diff_drive_patrol',
            name='diff_drive_patrol',
            output='screen',
            parameters=[{
                'forward_speed': 0.18,
                'turn_speed': 0.45,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    actions = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    actions.extend(robot_group(name, x, y, yaw, use_sim_time) for name, x, y, yaw in ROBOTS)
    return LaunchDescription(actions)
