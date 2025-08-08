from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    amcl_launch_path = os.path.join(
        get_package_share_directory('lidar_mapping'),
        'launch',
        'amcl_localization.launch.py'
    )
    ekf_amcl_params_path = os.path.join(
        get_package_share_directory('lidar_mapping'),
        'params',
        'ekf_amcl.yaml'
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
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='screen',
                arguments=['-3.499', '-1.924', '0.001', '0.000', '-0.000', '-0.709', '0.705', 'map', 'odom'],
                parameters=[{'use_sim_time': False}]
            ),
            Node(
                package='smart_rover',
                executable='transform_localization',
                name='transform_localization',
                parameters=[{'use_sim_time': False}]
            ),
            # Node(
            #     package='robot_localization',
            #     executable='ekf_node',
            #     name='ekf_filter_node',
            #     output='screen',
            #     parameters=[ekf_amcl_params_path]
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(amcl_launch_path)
            ),
        ])
    ])
