from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'manipulator_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [join('resource', package_name)]),
        (join('share', package_name), ['package.xml', 'README.md']),
        (join('share', package_name, 'config'), glob(join('config', '*.yaml'))),
        (join('share', package_name, 'launch'), glob(join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Sample Maintainers',
    maintainer_email='dev@example.com',
    description='Dependency-light ROS 2 planar manipulator simulation samples.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_simulator = manipulator_sim.manipulator_simulator:main',
            'target_commander = manipulator_sim.target_commander:main',
            'ik_target_commander = manipulator_sim.ik_target_commander:main',
        ],
    },
)
