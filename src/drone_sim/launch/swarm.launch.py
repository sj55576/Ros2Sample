"""Launch a small namespaced swarm of simulated drones."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _spawn_swarm(context):
    drone_count = int(LaunchConfiguration('drone_count').perform(context))
    spacing_m = float(LaunchConfiguration('spacing_m').perform(context))
    altitude_m = float(LaunchConfiguration('altitude_m').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = []
    for index in range(drone_count):
        namespace = f'drone_{index + 1}'
        x = (index % 3) * spacing_m
        y = (index // 3) * spacing_m
        actions.append(
            GroupAction([
                PushRosNamespace(namespace),
                Node(
                    package='drone_sim',
                    executable='sim_drone',
                    name='sim_drone',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'frame_id': 'odom',
                        'base_frame_id': f'{namespace}/base_link',
                        'initial_x': x,
                        'initial_y': y,
                        'initial_z': altitude_m,
                    }],
                ),
                Node(
                    package='drone_sim',
                    executable='waypoint_commander',
                    name='waypoint_commander',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'frame_id': 'odom',
                        'loop': True,
                        'waypoints': [
                            x, y, altitude_m,
                            x + spacing_m, y, altitude_m + 0.4,
                            x + spacing_m, y + spacing_m, altitude_m,
                            x, y + spacing_m, altitude_m + 0.2,
                        ],
                    }],
                ),
            ])
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_count', default_value='3'),
        DeclareLaunchArgument('spacing_m', default_value='2.0'),
        DeclareLaunchArgument('altitude_m', default_value='1.2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_spawn_swarm),
    ])
