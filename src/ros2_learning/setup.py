"""ros2_learning パッケージのセットアップスクリプト。"""

from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'ros2_learning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [join('resource', package_name)],
        ),
        (join('share', package_name), ['package.xml']),
        (
            join('share', package_name, 'config'),
            glob(join('config', '*.yaml')),
        ),
        (
            join('share', package_name, 'launch'),
            glob(join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Sample Maintainers',
    maintainer_email='dev@example.com',
    description='ROS 2 の基礎概念を段階的に学ぶための軽量チュートリアルパッケージ。',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher'
            ' = ros2_learning.minimal_publisher:main',
            'minimal_subscriber'
            ' = ros2_learning.minimal_subscriber:main',
            'minimal_service_server'
            ' = ros2_learning.minimal_service_server:main',
            'minimal_service_client'
            ' = ros2_learning.minimal_service_client:main',
            'minimal_action_server'
            ' = ros2_learning.minimal_action_server:main',
            'minimal_action_client'
            ' = ros2_learning.minimal_action_client:main',
            'parameter_demo'
            ' = ros2_learning.parameter_demo:main',
            'tf_broadcaster_demo'
            ' = ros2_learning.tf_broadcaster_demo:main',
            'tf_listener_demo'
            ' = ros2_learning.tf_listener_demo:main',
            'lifecycle_demo'
            ' = ros2_learning.lifecycle_demo:main',
        ],
    },
)
