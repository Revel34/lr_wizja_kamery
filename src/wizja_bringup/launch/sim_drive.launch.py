"""Drive the simulated rover with the keyboard and watch the depth-camera obstacle detection in RViz.

    terminal 1:  ros2 launch wizja_bringup sim_drive.launch.py
    terminal 2:  ros2 run teleop_twist_keyboard teleop_twist_keyboard

teleop keys: i = forward, , = back, j / l = turn, Shift+J / Shift+L = sideways (4-wheel steering),
k = stop, q / z = faster / slower. The teleop terminal must have focus while driving.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('wizja_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim_test.launch.py')),
            launch_arguments={
                'drive': 'true',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': os.path.join(bringup, 'rviz', 'wizja_drive.rviz'),
            }.items(),
        ),
        LogInfo(msg='[wizja] Drive with:  ros2 run teleop_twist_keyboard teleop_twist_keyboard  '
                    '(in a second terminal; Shift+J/L = sideways)'),
    ])
