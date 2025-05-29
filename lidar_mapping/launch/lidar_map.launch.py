from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                '/home/manuelo247/8vo/ros2_ws/src/lidar_mapping/params/nav2_params.yaml',
                {'yaml_filename': '/home/manuelo247/8vo/ros2_ws/src/path_planning/resource/ros2_custom_map.yaml'}
            ]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/home/manuelo247/8vo/ros2_ws/src/lidar_mapping/params/nav2_params.yaml']
        ),
    ])