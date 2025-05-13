from setuptools import setup
from glob import glob
import os

package_name = 'smart_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linearization = smart_rover.feedback_linearization_node:main',
            'circular = smart_rover.circular_trajectory:main',
            'rover_simulator = smart_rover.odom_simulator:main',
            'angle = smart_rover.angle_controller:main',
            'rdk_x3_robot_static = smart_rover.state_publisher_static:main',
            'path_follower = smart_rover.path_follower:main',
        ],
    },
)
