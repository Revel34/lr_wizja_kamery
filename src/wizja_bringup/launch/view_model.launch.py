"""Show the rover 3.0 model with the depth-camera cluster in RViz (no cameras / data needed).

    ros2 launch wizja_bringup view_model.launch.py
    ros2 launch wizja_bringup view_model.launch.py cameras_config:=/path/to/other_cameras.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wizja_bringup import camera_config
from wizja_bringup.launch_helpers import description_nodes


def _setup(context):
    # nominal frames on: there is no driver, so the URDF shows the depth optical frames too.
    return description_nodes(
        LaunchConfiguration('cameras_config').perform(context), 'full', nominal_camera_frames=True)


def generate_launch_description():
    bringup = get_package_share_directory('wizja_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config', default_value=camera_config.default_config_path()),
        OpaqueFunction(function=_setup),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup, 'rviz', 'wizja.rviz')],
            output='log',
        ),
    ])
