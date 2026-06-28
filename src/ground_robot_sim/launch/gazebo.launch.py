"""Launch ground_robot_sim in Gazebo with ros_gz bridge and obstacle avoidance."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('ground_robot_sim')
    gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(share_dir, 'worlds', 'default.sdf')
    urdf = os.path.join(share_dir, 'urdf', 'ground_robot_gazebo.urdf')

    with open(urdf, 'r') as f:
        robot_description = f.read()

    use_gui = LaunchConfiguration('use_gui')
    gz_version = LaunchConfiguration('gz_version')
    start_controller = LaunchConfiguration('start_controller')

    return LaunchDescription([
        # Software rendering for WSL2 (no GPU passthrough)
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),

        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Launch Gazebo GUI (false for headless / WSL2)',
        ),
        DeclareLaunchArgument(
            'gz_version',
            default_value='8',
            description='Gazebo Sim major version passed to ros_gz_sim',
        ),
        DeclareLaunchArgument(
            'start_controller',
            default_value='true',
            description='Start the diff_drive_patrol demo controller',
        ),

        # Gazebo Sim server-only (-s) for WSL2: no GUI, no rendering required
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': PythonExpression([
                    '"-s -r ' + world + '"'
                    ' if "false" == "', use_gui, '"'
                    ' else "-r ' + world + '"'
                ]),
                'gz_version': gz_version,
            }.items(),
        ),

        # robot_state_publisher (publishes /robot_description and TF for joints)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            arguments=[
                '-name', 'ground_robot',
                '-topic', '/robot_description',
                '-z', '0.1',
            ],
            output='screen',
        ),

        # ros_gz_bridge: Gazebo <-> ROS 2
        # Note: /scan bridge omitted (Sensors plugin requires GPU rendering)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen',
        ),

        # Controller: open-loop patrol (no scan required)
        Node(
            package='ground_robot_sim',
            executable='diff_drive_patrol',
            name='diff_drive_patrol',
            output='screen',
            condition=IfCondition(start_controller),
        ),
    ])
