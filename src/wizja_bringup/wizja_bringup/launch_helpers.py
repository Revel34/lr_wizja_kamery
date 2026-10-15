"""Small helpers shared by the wizja launch files."""

import math
import os
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from wizja_bringup.camera_config import RigConfig


def dds_env(enabled: bool) -> Dict[str, str]:
    """Fast-DDS profile: UDP transport only, data-sharing off.

    Same workaround as rover 2.2 (kick-autonomy rover_perception): avoids the Fast-DDS
    shared-memory / data-sharing busy-loop with late-joining readers of large messages
    (eProsima/Fast-DDS#6338). Has no effect when another RMW (e.g. CycloneDDS) is used.
    """
    if not enabled:
        return {}
    profile = os.path.join(
        get_package_share_directory('wizja_bringup'), 'config', 'fastdds_no_shm.xml')
    return {'FASTRTPS_DEFAULT_PROFILES_FILE': profile}


def _fmt(value: float) -> str:
    return f'{value:.6f}'


def static_camera_tf_nodes(rig: RigConfig, with_optical_frames: bool = False) -> List[Node]:
    """static_transform_publisher base_link -> camera_<name>_link for every camera.

    with_optical_frames additionally publishes camera_<name>_link -> camera_<name>_depth_optical_frame,
    which the real realsense driver does itself (publish_tf: true); only the simulator needs it.
    """
    nodes = []
    for cam in rig.cameras:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{cam.node_name}_mount_tf',
            arguments=[
                '--x', _fmt(cam.x), '--y', _fmt(cam.y), '--z', _fmt(cam.z),
                '--roll', _fmt(cam.roll), '--pitch', _fmt(cam.pitch), '--yaw', _fmt(cam.yaw),
                '--frame-id', rig.target_frame, '--child-frame-id', cam.link_frame,
            ],
            output='log',
        ))
        if with_optical_frames:
            # Optical frame: Z forward, X right, Y down (REP-103 camera convention).
            nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{cam.node_name}_optical_tf',
                arguments=[
                    '--roll', _fmt(-math.pi / 2), '--pitch', '0', '--yaw', _fmt(-math.pi / 2),
                    '--frame-id', cam.link_frame, '--child-frame-id', cam.depth_optical_frame,
                ],
                output='log',
            ))
    return nodes
