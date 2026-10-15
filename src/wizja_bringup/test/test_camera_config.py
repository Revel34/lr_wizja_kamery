import math
import os

import numpy as np

from wizja_bringup import camera_config

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'cameras.yaml')


def test_default_config_has_three_cameras():
    rig = camera_config.load(CONFIG)
    assert rig.target_frame == 'base_link'
    assert [c.name for c in rig.cameras] == ['front', 'left', 'right']
    assert rig.realsense['pointcloud.enable'] is True


def test_names_and_topics():
    cam = camera_config.load(CONFIG).cameras[0]
    assert cam.link_frame == 'camera_front_link'
    assert cam.depth_optical_frame == 'camera_front_depth_optical_frame'
    assert cam.cloud_topic == '/camera_front/depth/color/points'


def test_fusion_parameters_match_cameras():
    params = camera_config.fusion_parameters(camera_config.load(CONFIG))
    assert params['camera_names'] == ['front', 'left', 'right']
    assert params['cameras.left.topic'] == '/camera_left/depth/color/points'


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
