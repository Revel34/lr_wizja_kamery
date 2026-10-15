"""RealSense D455 drivers + static mounting transforms for every camera in cameras.yaml.

    ros2 launch wizja_bringup cameras.launch.py serial_front:=123456789012 ...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wizja_bringup import camera_config
from wizja_bringup.launch_helpers import dds_env, static_camera_tf_nodes


def _setup(context):
    rig = camera_config.load(LaunchConfiguration('cameras_config').perform(context))
    env = dds_env(LaunchConfiguration('use_fastdds_udp_profile').perform(context) == 'true')
    actions = []

    for cam in rig.cameras:
        serial = LaunchConfiguration(f'serial_{cam.name}', default=cam.serial_no).perform(context)
        serial = serial.strip().lstrip('_')
        params = dict(rig.realsense)
        params['camera_name'] = cam.node_name
        if serial:
            # Leading '_' keeps an all-digit serial a string; the driver strips it.
            params['serial_no'] = '_' + serial
        elif len(rig.cameras) > 1:
            actions.append(LogInfo(msg=(
                f'[wizja] WARNING: camera "{cam.name}" has no serial_no - with several cameras '
                'connected the drivers will grab devices in arbitrary order! Set serial_no in '
                'cameras.yaml or pass serial_' + cam.name + ':=<serial>.')))

        actions.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=cam.node_name,
            namespace='',
            parameters=[params],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=3.0,
            additional_env=env,
        ))

    if LaunchConfiguration('publish_camera_tf').perform(context) == 'true':
        actions.extend(static_camera_tf_nodes(rig))
    return actions


def generate_launch_description():
    # Serial arguments are declared for the cameras of the default config.
    try:
        default_rig = camera_config.load()
        serial_args = [
            DeclareLaunchArgument(
                f'serial_{cam.name}', default_value=cam.serial_no,
                description=f'Serial number of the {cam.name} D455 (rs-enumerate-devices -s)')
            for cam in default_rig.cameras
        ]
    except (OSError, ValueError):
        serial_args = []

    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config', default_value=camera_config.default_config_path(),
            description='Camera list, serial numbers and mounting poses'),
        DeclareLaunchArgument(
            'publish_camera_tf', default_value='true',
            description='Publish base_link -> camera_<name>_link from cameras.yaml. Set false if '
                        'the rover URDF (robot_state_publisher) already contains the cameras.'),
        DeclareLaunchArgument(
            'use_fastdds_udp_profile', default_value='true',
            description='Run with the UDP-only Fast-DDS profile (see config/fastdds_no_shm.xml)'),
        *serial_args,
        OpaqueFunction(function=_setup),
    ])
