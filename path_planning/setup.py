from setuptools import setup
import os
from glob import glob

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Usa lista explícita en lugar de find_packages
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', [
            'resource/ros2_custom_map.yaml',
            'resource/ros2_custom_map.pgm'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuelo247',
    maintainer_email='manuel8a31@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'planning = path_planning.planning_node:main',
        ],
    },
)