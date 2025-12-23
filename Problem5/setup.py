from setuptools import setup
import os
from glob import glob

package_name = 'exercice5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='22013326@CAMPUS',
    maintainer_email='student@example.com',
    description='Exercice 5 - Convoy with Best_Response and JESP algorithms',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = exercice5.robot_controller:main',
            'convoy_manager = exercice5.convoy_manager:main',
            'best_response = exercice5.best_response:main',
            'jesp = exercice5.jesp:main',
            'robot_markers = exercice5.robot_markers:main',
        ],
    },
)
