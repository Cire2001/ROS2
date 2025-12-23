# launch/exercice3.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'exercice3'

    # Utiliser la même carte que pour l'exercice 1
    map_file_path = os.path.join(
        get_package_share_directory('exercice3'),  # Utiliser la carte de exercice1
        'maps',
        'maze.yaml'
    )

    # Créer un fichier RViz spécifique pour l'exercice 3 si nécessaire
    # ou utiliser celui de l'exercice 1
    rviz_config_file = os.path.join(
        get_package_share_directory('exercice3'),
        'rviz',
        'exercice1.rviz'
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

        # Contrôleur de robot
        Node(
            package='exercice3',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),

        # Gestionnaire de convoi
        Node(
            package='exercice3',
            executable='convoy_manager',
            name='convoy_manager',
            output='screen'
        )
    ])
