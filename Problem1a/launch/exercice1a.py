from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'exercice1a'

    map_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'maps',
        'maze.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'exercice1a.rviz'
    )

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}],
    )

    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # Robot Controller - Launch after a delay
    robot_controller_delayed = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            Node(
                package=pkg_name,
                executable='robot_controller',
                name='robot_controller',
                output='screen'
            )
        ]
    )

    # Destination Manager - Launch after a delay
    destination_manager_delayed = TimerAction(
        period=3.0,  # 3 second delay (after the robot controller)
        actions=[
            Node(
                package=pkg_name,
                executable='destination_manager',
                name='destination_manager',
                output='screen'
            )
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node,
        robot_controller_delayed,
        destination_manager_delayed,
        rviz_node
    ])
