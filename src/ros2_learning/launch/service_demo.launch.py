"""サービスサーバーとサービスクライアントを同時に起動するランチファイル。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """minimal_service_server と minimal_service_client を起動する。"""
    # サービスサーバーノードの定義
    server_node = Node(
        package='ros2_learning',
        executable='minimal_service_server',
        name='minimal_service_server',
        output='screen',
    )

    # サービスクライアントノードの定義
    client_node = Node(
        package='ros2_learning',
        executable='minimal_service_client',
        name='minimal_service_client',
        output='screen',
    )

    return LaunchDescription([
        server_node,
        client_node,
    ])
