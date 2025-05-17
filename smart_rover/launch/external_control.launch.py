from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_rover',
            executable='angle',
            name='angle'
        ),
        Node(
            package='smart_rover',
            executable='linearization',
            name='linearization'
        ),
        Node(
            package='smart_rover',
            executable='path_follower',
            name='path_follower'
        ),
        Node(
            package='smart_rover',
            executable='switch_controller',
            name='switch_controller'
        ),
        Node(
            package='path_planning',
            executable='planning',
            name='planning'
        ),
        Node(
            package='path_planning',
            executable='splines',
            name='splines'
        ),
    ])