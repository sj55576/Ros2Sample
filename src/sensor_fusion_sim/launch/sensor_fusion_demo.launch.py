"""Launch sensor fusion demo: noisy sensors, filter, recorder."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
import launch.events
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Return LaunchDescription for the sensor fusion demo."""
    share_dir = get_package_share_directory('sensor_fusion_sim')
    config_file = join(share_dir, 'config', 'sensor_fusion.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    recorder_node = LifecycleNode(
        package='sensor_fusion_sim',
        executable='lifecycle_data_recorder',
        name='lifecycle_data_recorder',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                recorder_node
            ),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                recorder_node
            ),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        Node(
            package='sensor_fusion_sim',
            executable='noisy_sensor_node',
            name='noisy_sensor_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='sensor_fusion_sim',
            executable='complementary_filter_node',
            name='complementary_filter',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='sensor_fusion_sim',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        recorder_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=recorder_node,
                on_start=[configure_event],
            ),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=recorder_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[activate_event],
            ),
        ),
    ])
