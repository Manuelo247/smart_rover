from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_rover',
            executable='switch_controller',
            name='switch_controller'
        ),
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
    ])