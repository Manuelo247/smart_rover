from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Ajusta la ruta del mapa según corresponda a tu workspace
    map_path = os.path.join(
        os.getenv('ROS_PACKAGE_PATH', '').split(':')[0],
        'smart_rover', 'resource', 'mapa_ines.yaml'
    )

    return LaunchDescription([
        Node(
            package='smart_rover',
            executable='path_follower',
            name='path_follower'
        ),
        Node(
           package='path_planning',
           executable='planning',
           name='planning',
           parameters=[{'map_filename': map_path, 'use_sim_time': True}],
        ),
    ])