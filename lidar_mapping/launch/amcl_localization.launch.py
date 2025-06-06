from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_path = "/home/manuelo247/8vo/ros2_ws/src/smart_rover/resource/mapa_ines.yaml"
    
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
        Node(
            package='lidar_mapping',
            executable='initialpose',
            name='initialpose'
        )
    ])