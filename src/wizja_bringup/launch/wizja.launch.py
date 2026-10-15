"""Full depth-camera perception on the rover: D455 drivers + fusion + obstacle detection.

    ros2 launch wizja_bringup wizja.launch.py
    ros2 launch wizja_bringup wizja.launch.py use_rviz:=true serial_front:=... serial_left:=...
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('wizja_bringup')
    launch_dir = os.path.join(bringup, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cameras.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'perception.launch.py'))),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'wizja.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='log',
        ),
    ])
