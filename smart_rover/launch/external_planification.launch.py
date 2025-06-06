from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta absoluta al archivo de mapa en smart_rover/maps
    map_path = os.path.join(
        os.getenv('ROS_PACKAGE_PATH', ''),
        'smart_rover', 'resource', 'mapa_ines.yaml'
    )

    control_launch_path = os.path.join(
        os.path.dirname(__file__), 'control.launch.py'
    )

    localization_launch_path = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'localization.launch.py'
    )
    
    path_planning_launch_path = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'localization.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_planning_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path)
        ),
    ])
