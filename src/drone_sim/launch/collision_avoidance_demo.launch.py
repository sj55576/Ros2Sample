"""
Launch a small swarm with reactive collision avoidance enabled.

``drone_1`` runs a ``collision_avoidance`` node between its
``waypoint_commander`` and ``sim_drone``: the commander's raw goal is
nudged away from the other drones before ``sim_drone`` ever sees it.
``drone_2`` and ``drone_3`` fly their own waypoint loops unmodified, so
they act as moving traffic that ``drone_1`` must steer around.
"""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Return a LaunchDescription for the collision avoidance demo scenario."""
    share_dir = get_package_share_directory('drone_sim')
    config_file = join(share_dir, 'config', 'collision_avoidance.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        GroupAction([
            PushRosNamespace('drone_1'),
            Node(
                package='drone_sim',
                executable='sim_drone',
                name='sim_drone',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame_id': 'drone_1/base_link',
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_z': 1.0,
                }],
            ),
            Node(
                package='drone_sim',
                executable='waypoint_commander',
                name='waypoint_commander',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints': [
                        0.0, 0.0, 1.0,
                        4.0, 0.0, 1.0,
                        4.0, 4.0, 1.0,
                        0.0, 4.0, 1.0,
                    ],
                    'loop': True,
                }],
                remappings=[('setpoint_pose', 'raw_setpoint_pose')],
            ),
            Node(
                package='drone_sim',
                executable='collision_avoidance',
                name='collision_avoidance',
                output='screen',
                parameters=[config_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('setpoint_pose', 'raw_setpoint_pose'),
                    ('adjusted_setpoint_pose', 'setpoint_pose'),
                ],
            ),
        ]),
        GroupAction([
            PushRosNamespace('drone_2'),
            Node(
                package='drone_sim',
                executable='sim_drone',
                name='sim_drone',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame_id': 'drone_2/base_link',
                    'initial_x': 4.0,
                    'initial_y': 0.0,
                    'initial_z': 1.0,
                }],
            ),
            Node(
                package='drone_sim',
                executable='waypoint_commander',
                name='waypoint_commander',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints': [
                        4.0, 0.0, 1.0,
                        0.0, 0.0, 1.0,
                        0.0, 4.0, 1.0,
                        4.0, 4.0, 1.0,
                    ],
                    'loop': True,
                }],
            ),
        ]),
        GroupAction([
            PushRosNamespace('drone_3'),
            Node(
                package='drone_sim',
                executable='sim_drone',
                name='sim_drone',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame_id': 'drone_3/base_link',
                    'initial_x': 2.0,
                    'initial_y': 2.0,
                    'initial_z': 1.0,
                }],
            ),
            Node(
                package='drone_sim',
                executable='waypoint_commander',
                name='waypoint_commander',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints': [
                        2.0, 2.0, 1.0,
                        2.0, 2.0, 2.0,
                    ],
                    'loop': True,
                }],
            ),
        ]),
    ])
