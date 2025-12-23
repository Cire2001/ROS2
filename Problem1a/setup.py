from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'exercice1a'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # for ros2 pkg index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # all launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # maps and rviz
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='22013326@CAMPUS',
    maintainer_email='22013326@CAMPUS@todo.todo',
    description='Robot rendezvous at a common destination',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = exercice1a.robot_controller:main',
            'destination_manager = exercice1a.destination_manager:main',
            'random_robot = exercice1a.random_robot:main',
        ],
    },
)
