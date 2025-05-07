from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    yahboomcar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'yahboomcar_bringup_launch.py')
        )
    )

    smart_rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('smart_rover'), 'rdk_x3_model.launch.py')
        )
    )

    oradar_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oradar_lidar'), 'ms200_scan.launch.py')
        )
    )

    return LaunchDescription([
        yahboomcar_launch,
        smart_rover_launch,
        oradar_lidar_launch
    ])
