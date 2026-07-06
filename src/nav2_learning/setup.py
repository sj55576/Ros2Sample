"""nav2_learning パッケージのセットアップスクリプト."""
from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'nav2_learning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [join('resource', package_name)]),
        (join('share', package_name), ['package.xml']),
        (join('share', package_name, 'config'), glob(join('config', '*.yaml'))),
        (join('share', package_name, 'launch'), glob(join('launch', '*.launch.py'))),
        (join('share', package_name, 'maps'), glob(join('maps', '*'))),
        (join('share', package_name, 'rviz'), glob(join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Sample Maintainers',
    maintainer_email='dev@example.com',
    description='Navigation2 の概念を段階的に学ぶための学習パッケージ。',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_map_publisher = nav2_learning.simple_map_publisher:main',
            'simple_path_planner = nav2_learning.simple_path_planner:main',
            'simple_path_follower = nav2_learning.simple_path_follower:main',
            'nav2_waypoint_client = nav2_learning.nav2_waypoint_client:main',
            'costmap_monitor = nav2_learning.costmap_monitor:main',
            'simple_occupancy_mapper = nav2_learning.simple_occupancy_mapper:main',
        ],
    },
)
