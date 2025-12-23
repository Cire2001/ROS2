# launch/exercice5.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'exercice5'

    # Utiliser la carte du labyrinthe
    map_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'maps',
        'maze.yaml'
    )

    # Utiliser la configuration RViz de l'exercice 5
    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'exercice5.rviz'
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
            package='exercice5',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),

        # Gestionnaire de convoi
        # MODIFIÉ: Changé le nom pour correspondre à l'entrée dans setup.py
        Node(
            package='exercice5',
            executable='convoy_manager',
            name='convoy_manager',
            output='screen'
        ),

        # Best Response Algorithm
        Node(
            package='exercice5',
            executable='best_response',
            name='best_response',
            output='screen'
        ),

        # JESP Algorithm
        Node(
            package='exercice5',
            executable='jesp',
            name='jesp',
            output='screen'
        ),

        # Robot Markers
        Node(
            package='exercice5',
            executable='robot_markers',
            name='robot_markers',
            output='screen'
        )
    ])
