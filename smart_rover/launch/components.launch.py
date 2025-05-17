import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Paths a los archivos de lanzamiento individuales
    yahboomcar_launch = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'launch',
        'yahboomcar_bringup_launch.py'
    )

    #smart_rover_launch = os.path.join(
    #    get_package_share_directory('smart_rover'),
    #    'launch',
    #    'rdk_x3_model.launch.py'
    #)

    lidar_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )

    map_gmapping_launch = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'launch',
        'slam_gmapping.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yahboomcar_launch)
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(smart_rover_launch)
        #),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_gmapping_launch)
        ),
    ])
