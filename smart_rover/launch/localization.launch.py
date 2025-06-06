from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    amcl_launch_path = os.path.join(
        get_package_share_directory('lidar_mapping'),
        'launch',
        'amcl_localization.launch.py'
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['-3.499', '1.924', '0.001', '0.000', '-0.000', '-0.709', '0.705', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='smart_rover',
            executable='transform_localization',
            name='transform_localization',
            parameters=[{'use_sim_time': True}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_launch_path)
        ),
    ])