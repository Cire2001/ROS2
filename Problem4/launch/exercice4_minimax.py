# exercice4_minimax.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'exercice4'
    pkg_dir = get_package_share_directory(pkg_name)

    # Chemin direct vers les fichiers de configuration
    map_file_path = os.path.join(pkg_dir, 'maps', 'maze.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'exercice4.rviz')

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

        # Surveillance Robot 1
        Node(
            package='exercice4',
            executable='surveillance_robot',
            name='surveillance_robot_1',
            output='screen',
            arguments=['1']
        ),

        # Surveillance Robot 2
        Node(
            package='exercice4',
            executable='surveillance_robot',
            name='surveillance_robot_2',
            output='screen',
            arguments=['2']
        ),

        # Minimax Planner
        Node(
            package='exercice4',
            executable='minimax_planner',
            name='minimax_planner',
            output='screen'
        )
    ])
