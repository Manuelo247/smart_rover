import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Paths a los archivos de lanzamiento individuales
    components_launch = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'components.launch.py'
    )

    external_control_launch = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'external_control.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(components_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(external_control_launch)
        ),
    ])