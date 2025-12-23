from setuptools import setup
import os
from glob import glob

package_name = 'exercice4'

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
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),  # Changé de 'config' à 'rviz'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Exercice 4 - Robot Surveillance',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'surveillance_robot = exercice4.surveillance_robot:main',
            'joint_planner = exercice4.joint_planner:main',
            'minimax_planner = exercice4.minimax_planner:main',
        ],
    },
)
