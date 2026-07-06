from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'sensor_fusion_sim'

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
    description=(
        'Sensor fusion demo with lifecycle nodes, '
        'QoS profiles, and callback groups.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noisy_sensor_node = '
            'sensor_fusion_sim.noisy_sensor_node:main',
            'complementary_filter_node = '
            'sensor_fusion_sim.complementary_filter_node:main',
            'lifecycle_data_recorder = '
            'sensor_fusion_sim.lifecycle_data_recorder:main',
            'ekf_node = '
            'sensor_fusion_sim.ekf_node:main',
        ],
    },
)
