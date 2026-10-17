import math
import os

import numpy as np
import pytest

from wizja_bringup import camera_config

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'cameras.yaml')


def test_default_config_has_three_cameras():
    rig = camera_config.load(CONFIG)
    assert rig.target_frame == 'base_footprint'
    assert rig.parent_frame == 'base_link'
    assert [c.name for c in rig.cameras] == ['front', 'left', 'right']
    assert rig.realsense['pointcloud.enable'] is True


def test_serials_are_twelve_digit_strings():
    serials = {c.name: c.serial_no for c in camera_config.load(CONFIG).cameras}
    assert serials == {'front': '254122303704', 'left': '253822300711', 'right': '253822300777'}


def test_names_and_topics():
    cams = {c.name: c for c in camera_config.load(CONFIG).cameras}
    left = cams['left']
    assert left.source == 'driver' and not left.external
    assert left.link_frame == 'camera_left_link'
    assert left.depth_optical_frame == 'camera_left_depth_optical_frame'
    assert left.cloud_topic == '/camera_left/depth/color/points'


def test_front_is_the_shared_main_stack_camera():
    front = {c.name: c for c in camera_config.load(CONFIG).cameras}['front']
    assert front.external
    # Frame names of the main stack's driver / rover30 URDF.
    assert front.link_frame == 'camera_link'
    assert front.depth_optical_frame == 'camera_depth_optical_frame'
    assert front.depth_topic == '/camera/realsense_camera/depth/image_rect_raw'
    assert front.info_topic == '/camera/realsense_camera/depth/camera_info'
    # rover30 URDF camera_joint (0.42, 0, 0.18) + D455 bottom screw -> camera_link offset.
    assert (front.x, front.y, front.z) == (0.43115, 0.0475, 0.1945)
    assert (front.roll, front.pitch, front.yaw) == (0.0, 0.0, 0.0)


def test_external_camera_needs_topics(tmp_path):
    path = tmp_path / 'cams.yaml'
    path.write_text('cameras:\n  front:\n    source: external\n')
    with pytest.raises(ValueError):
        camera_config.load(str(path))


def test_fusion_parameters_match_cameras():
    params = camera_config.fusion_parameters(camera_config.load(CONFIG))
    assert params['camera_names'] == ['front', 'left', 'right']
    assert params['cameras.left.type'] == 'cloud'
    assert params['cameras.left.topic'] == '/camera_left/depth/color/points'
    assert params['cameras.front.type'] == 'depth_image'
    assert params['cameras.front.topic'] == '/camera/realsense_camera/depth/image_rect_raw'
    assert params['cameras.front.info_topic'] == '/camera/realsense_camera/depth/camera_info'
    assert params['cameras.front.decimation'] == 2


def test_numeric_serial_stays_string(tmp_path):
    path = tmp_path / 'cams.yaml'
    path.write_text('cameras:\n  front:\n    serial_no: 123456789012\n')
    rig = camera_config.load(str(path))
    assert rig.cameras[0].serial_no == '123456789012'


def test_rotation_convention():
    # pitch > 0 tilts the camera X axis down.
    r = np.array(camera_config.rotation_matrix(0.0, 0.35, 0.0))
    forward = r @ np.array([1.0, 0.0, 0.0])
    assert forward[2] < 0.0
    assert math.isclose(math.asin(-forward[2]), 0.35, abs_tol=1e-9)
    # yaw = +90 deg turns X into +Y (left camera looks left), pitch still tilts it down.
    r = np.array(camera_config.rotation_matrix(0.0, 0.35, math.pi / 2))
    forward = r @ np.array([1.0, 0.0, 0.0])
    assert forward[1] > 0.9 and forward[2] < 0.0 and abs(forward[0]) < 1e-9
