from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [join('resource', package_name)]),
        (join('share', package_name), ['package.xml', 'README.md']),
        (join('share', package_name, 'config'), glob(join('config', '*.yaml'))),
        (join('share', package_name, 'launch'), glob(join('launch', '*.launch.py'))),
        (join('share', package_name, 'rviz'), glob(join('rviz', '*.rviz'))),
        (join('share', package_name, 'urdf'), glob(join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Sample Maintainers',
    maintainer_email='dev@example.com',
    description='Dependency-light Python ROS 2 quadrotor simulation samples.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'altitude_hold = drone_sim.altitude_hold:main',
            'battery_monitor = drone_sim.battery_monitor:main',
            'emergency_land = drone_sim.emergency_land:main',
            'sim_drone = drone_sim.sim_drone:main',
            'waypoint_commander = drone_sim.waypoint_commander:main',
        ],
    },
)
