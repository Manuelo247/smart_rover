from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Ruta absoluta al archivo de mapa en smart_rover/maps
    map_path = os.path.join(
        os.getenv('ROS_PACKAGE_PATH', ''),  # Busca en los paquetes ROS
        'smart_rover', 'resource', 'mapa_ines.yaml'
    )

    # Alternativamente, si sabes la ruta relativa al workspace:
    # map_path = os.path.join(
    #     os.path.dirname(__file__), '..', 'maps', 'ros2_custom_map.yaml'
    # )

    return LaunchDescription([
        Node(
            package='smart_rover',
            executable='angle',
            name='angle'
        ),
        Node(
            package='smart_rover',
            executable='linearization',
            name='linearization'
        ),
        Node(
            package='smart_rover',
            executable='path_follower',
            name='path_follower'
        ),
        Node(
            package='smart_rover',
            executable='switch_controller',
            name='switch_controller'
        ),
        #Node(
        #    package='path_planning',
        #    executable='planning',
        #    name='planning',
        #    parameters=[{'map_filename': map_path}] 
        #),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
       ),
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'smart_rover', 'safety'],
        #     output='screen'
        # ),
    ])
