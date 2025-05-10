from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

map_file = os.path.join(
    get_package_share_directory('path_planning'),
    'resource',
    'ros2_custom_map.yaml'
)

parameters=[{
    'yaml_filename': map_file,
    'frame_id': 'map',
    'use_sim_time': False,
    'publish_periodically': True,  
    'publish_rate': 1.0           
}]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=parameters
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        )
    ])