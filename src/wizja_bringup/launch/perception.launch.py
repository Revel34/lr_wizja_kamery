"""Cloud fusion + obstacle detection, both in one multi-threaded component container
(intra-process, zero-copy hand-over of the fused cloud)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from wizja_bringup import camera_config
from wizja_bringup.launch_helpers import dds_env


def _setup(context):
    rig = camera_config.load(LaunchConfiguration('cameras_config').perform(context))
    params_file = LaunchConfiguration('perception_params').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    env = dds_env(LaunchConfiguration('use_fastdds_udp_profile').perform(context) == 'true')
    common = {'use_sim_time': use_sim_time}
    intra = [{'use_intra_process_comms': True}]

    return [ComposableNodeContainer(
        name='wizja_perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='wizja_perception',
                plugin='wizja_perception::CloudFusionNode',
                name='cloud_fusion',
                parameters=[params_file, camera_config.fusion_parameters(rig), common],
                extra_arguments=intra,
            ),
            ComposableNode(
                package='wizja_perception',
                plugin='wizja_perception::ObstacleDetectorNode',
                name='obstacle_detector',
                parameters=[params_file, common],
                extra_arguments=intra,
            ),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
        additional_env=env,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras_config', default_value=camera_config.default_config_path()),
        DeclareLaunchArgument(
            'perception_params',
            default_value=os.path.join(
                get_package_share_directory('wizja_perception'), 'config', 'perception.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_fastdds_udp_profile', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
