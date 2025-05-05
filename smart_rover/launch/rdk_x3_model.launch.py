import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    urdf_file = os.path.join(
        get_package_share_directory('smart_rover'),
        'urdf',
        'rdk_x3_robot.urdf'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('smart_rover'),
        'rviz',
        'rdk_conf.rviz'
    )

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='smart_rover',
            executable='rdk_x3_robot_static',
            name='static_state_publisher',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
