"""Hardware-free test: rover model + synthetic D455 clouds (fake_cameras) -> fusion -> detection.

    ros2 launch wizja_bringup sim_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wizja_bringup import camera_config
from wizja_bringup.launch_helpers import description_nodes


def _setup(context):
    path = LaunchConfiguration('cameras_config').perform(context)
    return [
        # Full rover model; the URDF also provides the camera optical frames that the real
        # realsense driver would publish.
        *description_nodes(path, 'full', nominal_camera_frames=True),
        Node(
            package='wizja_bringup',
            executable='fake_cameras.py',
            name='fake_cameras',
            parameters=[{
                'cameras_config': path,
                'rate': float(LaunchConfiguration('rate').perform(context)),
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('wizja_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config', default_value=camera_config.default_config_path()),
        DeclareLaunchArgument('rate', default_value='15.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=_setup),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'perception.launch.py'))),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'wizja.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='log',
        ),
    ])
