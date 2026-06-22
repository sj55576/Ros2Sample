"""Launch a formation flight demo with one leader and two followers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        GroupAction([
            PushRosNamespace('leader'),
            Node(
                package='drone_sim',
                executable='sim_drone',
                name='sim_drone',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame_id': 'leader/base_link',
                    'publish_rate_hz': 50.0,
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_z': 0.0,
                }],
            ),
            Node(
                package='drone_sim',
                executable='waypoint_commander',
                name='waypoint_commander',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints': [0.0, 0.0, 2.0, 4.0, 0.0, 2.0, 4.0, 4.0, 2.0, 0.0, 4.0, 2.0],
                    'tolerance_m': 0.3,
                    'hold_time_sec': 1.0,
                    'loop': True,
                }],
            ),
        ]),
        GroupAction([
            PushRosNamespace('follower_1'),
            Node(
                package='drone_sim',
                executable='sim_drone',
                name='sim_drone',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame_id': 'follower_1/base_link',
                    'publish_rate_hz': 50.0,
                    'initial_x': 2.0,
                    'initial_y': 0.0,
                    'initial_z': 0.0,
                }],
            ),
            Node(
                package='drone_sim',
                executable='formation_controller',
                name='formation_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'leader_odom_topic': '/leader/odom',
                    'offset_x': 2.0,
                    'offset_y': 0.0,
                    'offset_z': 0.0,
                    'smoothing_gain': 0.7,
                }],
            ),
        ]),
        GroupAction([
            PushRosNamespace('follower_2'),
            Node(
                package='drone_sim',
                executable='sim_drone',
                name='sim_drone',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame_id': 'follower_2/base_link',
                    'publish_rate_hz': 50.0,
                    'initial_x': 0.0,
                    'initial_y': 2.0,
                    'initial_z': 0.0,
                }],
            ),
            Node(
                package='drone_sim',
                executable='formation_controller',
                name='formation_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'leader_odom_topic': '/leader/odom',
                    'offset_x': 0.0,
                    'offset_y': 2.0,
                    'offset_z': 0.0,
                    'smoothing_gain': 0.7,
                }],
            ),
        ]),
    ])
