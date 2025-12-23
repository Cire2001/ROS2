# launch/exercice2.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'exercice2'

    # Utiliser la carte
    map_file_path = os.path.join(
        get_package_share_directory('exercice2'),
        'maps',
        'maze.yaml'
    )

    # Utiliser la configuration RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('exercice2'),
        'rviz',
        'exercice2.rviz'  # Utiliser exercice2.rviz au lieu de exercice1.rviz
    )

    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}],
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Ajouter les nœuds robot_controller et rendezvous_manager
        Node(
            package='exercice2',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),

        Node(
            package='exercice2',
            executable='rendezvous_manager',
            name='rendezvous_manager',
            output='screen'
        )
    ])
