from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Argumento para el namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot2',
        description='Namespace para todos los nodos'
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
        get_package_share_directory('path_planning'),
        'launch',
        'path_planning.launch.py'
    )

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path),
            launch_arguments={'namespace': namespace}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_planning_launch_path),
            launch_arguments={'namespace': namespace}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path),
            launch_arguments={'namespace': namespace}.items()
        ),
    ])
