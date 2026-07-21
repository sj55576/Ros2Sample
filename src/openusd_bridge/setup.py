from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'openusd_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [join('resource', package_name)],
        ),
        (join('share', package_name), ['package.xml', 'README.md']),
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
    description='Record ROS 2 odometry as time-sampled OpenUSD transforms.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_usd = openusd_bridge.odom_to_usd:main',
        ],
    },
)
