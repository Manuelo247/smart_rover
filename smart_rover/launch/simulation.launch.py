from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path al launch de rdk_x3_model
    rdk_x3_model_launch = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'rdk_x3_model.launch.py'
    )
    # Path al launch de external_control
    external_control_launch = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'external_control.launch.py'
    )

    return LaunchDescription([
        Node(
            package='smart_rover',
            executable='rover_simulator',
            name='rover_simulator',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', '-u', '/home/manuelo247/8vo/ros2_ws/src/path_planning/test/publish_map.py'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rdk_x3_model_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(external_control_launch)
        ),
    ])