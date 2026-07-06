"""parameter_demo ノードを設定ファイルから起動するランチファイル."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """parameter_demo ノードのランチディスクリプションを生成する."""
    # パッケージの共有ディレクトリからデフォルト設定ファイルのパスを取得する
    share_dir = get_package_share_directory('ros2_learning')
    default_config = join(share_dir, 'config', 'parameter_demo.yaml')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # 設定ファイルのパスを引数として渡せるようにする
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='パラメータ設定ファイルのパス',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='シミュレーション時刻を使用するかどうか',
        ),
        Node(
            package='ros2_learning',
            executable='parameter_demo',
            name='parameter_demo',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
