"""Small helpers shared by the wizja launch files."""

import os
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wizja_bringup import camera_config
from wizja_bringup.rover_geometry import check_cameras_in_urdf, expand_urdf


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


ROVER_MODEL_MODES = ('cameras_only', 'full', 'none')


def description_nodes(cameras_config: str, mode: str, nominal_camera_frames: bool = False,
                      use_sim_time: bool = False, joint_state_publisher: bool = True) -> List[Node]:
    """robot_state_publisher (+ joint_state_publisher) for the rover model with the cameras.

    mode:
      cameras_only  base_link -> camera mount + camera_<name>_link only. For the real rover, where the
                    main stack's robot_state_publisher already publishes the rover itself.
      full          whole rover30 model rooted at base_footprint (laptop, simulator, view_model).
      none          publish nothing (the rover's own URDF already contains the cameras).

    joint_state_publisher=False in 'full' mode when something else publishes /wizja/joint_states
    (the driving simulator animates the steer and wheel joints itself).

    Runs in namespace 'wizja' -> /wizja/robot_description and /wizja/joint_states, so it never
    clashes with the main stack's /robot_description. TF goes to the global /tf and /tf_static.
    """
    if mode not in ROVER_MODEL_MODES:
        raise ValueError(f'rover_model must be one of {ROVER_MODEL_MODES}, got {mode!r}')
    if mode == 'none':
        return []

    # cameras_config only decides which cameras must exist; their positions come from the URDF.
    urdf = expand_urdf(include_rover=(mode == 'full'), nominal_camera_frames=nominal_camera_frames)
    check_cameras_in_urdf(camera_config.load(cameras_config), urdf, include_external=(mode == 'full'))
    nodes = [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='wizja',
        parameters=[{
            'robot_description': ParameterValue(urdf, value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='log',
    )]
    if mode == 'full' and joint_state_publisher:
        # Publishes 0 for the steer/wheel joints so their links get a TF. Only in 'full' mode -
        # on the rover the real joint states come from the main stack.
        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace='wizja',
            parameters=[{'use_sim_time': use_sim_time}],
            output='log',
        ))
    return nodes
