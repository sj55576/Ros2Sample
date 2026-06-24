from glob import glob
from setuptools import find_packages, setup

package_name = 'ground_robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Sample Maintainers',
    maintainer_email='maintainer@example.com',
    description='Lightweight ROS 2 ground robot simulation samples.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics_publisher = ground_robot_sim.diagnostics_publisher:main',
            'diff_drive_patrol = ground_robot_sim.diff_drive_patrol:main',
            'ground_robot_node = ground_robot_sim.ground_robot_node:main',
            'lidar_obstacle_avoid = ground_robot_sim.lidar_obstacle_avoid:main',
            'lidar_obstacle_stop = ground_robot_sim.lidar_obstacle_stop:main',
            'navigate_waypoints_server = ground_robot_sim.navigate_waypoints_server:main',
            'waypoint_follower = ground_robot_sim.waypoint_follower:main',
        ],
    },
)
