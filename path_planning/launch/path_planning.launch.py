from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    map_path = os.path.join(
        os.getenv('ROS_PACKAGE_PATH', '').split(':')[0],
        'smart_rover', 'resource', 'tec_warehouse.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot2',
            description='Namespace para los nodos'
        ),
        GroupAction([
            # PushRosNamespace(namespace),
            Node(
                package='smart_rover',
                executable='path_follower',
                name='path_follower'
            ),
            Node(
                package='path_planning',
                executable='planning',
                name='planning',
                parameters=[{'map_filename': map_path, 'use_sim_time': False}],
            ),
        ])
    ])