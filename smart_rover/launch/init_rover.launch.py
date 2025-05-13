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

<<<<<<< HEAD
    #smart_rover_launch = os.path.join(
    #    get_package_share_directory('smart_rover'),
    #    'launch',
    #    'rdk_x3_model.launch.py'
    #)
=======
    smart_rover_launch = os.path.join(
        get_package_share_directory('smart_rover'),
        'launch',
        'rdk_x3_model.launch.py'
    )
>>>>>>> f057b10fe58973ad6050474130d37f36f99d8ce8

    lidar_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yahboomcar_launch)
        ),
<<<<<<< HEAD
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(smart_rover_launch)
        #),
=======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(smart_rover_launch)
        ),
>>>>>>> f057b10fe58973ad6050474130d37f36f99d8ce8
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),
    ])
