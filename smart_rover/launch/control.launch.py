from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot2',
            description='Namespace para los nodos'
        ),
        GroupAction([
            # PushRosNamespace(namespace),
            Node(
                package='smart_rover',
                executable='switch_controller',
                name='switch_controller',
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
    ])
