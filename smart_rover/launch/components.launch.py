import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    yahboomcar_launch = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'launch',
        'yahboomcar_bringup_launch.py'
    )

    lidar_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot2',
            description='Namespace para los nodos'
        ),
        GroupAction([
            # PushRosNamespace(namespace),  # <--- comenta o elimina esta línea
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yahboomcar_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch)
            ),
            # Agrega aquí más nodos o launches y todos estarán bajo el namespace
        ])
    ])
