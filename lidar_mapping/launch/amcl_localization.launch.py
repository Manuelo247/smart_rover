from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_server_launch_path = os.path.join(
        get_package_share_directory('lidar_mapping'),
        'launch',
        'map_server.launch.py'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_server_launch_path)
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                os.path.join(
                    get_package_share_directory('lidar_mapping'),  # O el paquete donde esté tu YAML
                    'params',
                    'amcl_params.yaml'
                )
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['amcl']
            }]
        ),
        Node(
            package='lidar_mapping',
            executable='initialpose',
            name='initialpose'
        )
    ])