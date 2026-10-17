import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from wizja_bringup import camera_config, rover_geometry

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'cameras.yaml')


@pytest.fixture(scope='module')
def full_urdf():
    return rover_geometry.expand_urdf(CONFIG, include_rover=True, nominal_camera_frames=True)


@pytest.fixture(scope='module')
def cameras_only_urdf():
    return rover_geometry.expand_urdf(CONFIG, include_rover=False, nominal_camera_frames=False)


def links(urdf):
    return {link.get('name') for link in ET.fromstring(urdf).findall('link')}


def root_link(urdf):
    root = ET.fromstring(urdf)
    children = {j.find('child').get('link') for j in root.findall('joint')}
    roots = links(urdf) - children
    assert len(roots) == 1
    return roots.pop()


def test_full_model(full_urdf):
    assert root_link(full_urdf) == 'base_footprint'
    names = links(full_urdf)
    for wheel in ('left_front', 'right_front', 'left_rear', 'right_rear'):
        assert f'{wheel}_wheel_link' in names
        assert f'{wheel}_steer_link' in names
    # The lidar was replaced by the cameras; the existing front D455 keeps its rover30 frame names.
    assert 'laser_frame' not in names
    assert 'camera_link' in names
    tf = rover_geometry.frame_transforms(full_urdf, 'base_footprint')
    assert tf['base_link'][2, 3] == pytest.approx(rover_geometry.BASE_LINK_HEIGHT)


def test_cameras_only_model(cameras_only_urdf):
    assert root_link(cameras_only_urdf) == 'base_link'
    names = links(cameras_only_urdf)
    assert not any('wheel' in n for n in names)
    assert 'base_footprint' not in names           # published by the main rover stack
    assert 'camera_link' not in names              # shared front camera: main stack's URDF has it
    assert {'camera_left_link', 'camera_right_link', 'camera_post_link'} <= names
    assert not any(n.endswith('optical_frame') for n in names)  # published by the realsense driver


def test_every_camera_present_with_yaml_pose(full_urdf):
    rig = camera_config.load(CONFIG)
    tf = rover_geometry.frame_transforms(full_urdf, rig.parent_frame)
    for cam in rig.cameras:
        m = tf[cam.link_frame]
        assert m[:3, 3] == pytest.approx([cam.x, cam.y, cam.z], abs=1e-9)
        expected = np.array(camera_config.rotation_matrix(cam.roll, cam.pitch, cam.yaw))
        assert np.allclose(m[:3, :3], expected, atol=1e-9)
        assert cam.depth_optical_frame in tf


def test_cameras_look_front_left_right(full_urdf):
    rig = camera_config.load(CONFIG)
    tf = rover_geometry.frame_transforms(full_urdf, rig.target_frame)
    headings = {}
    for cam in rig.cameras:
        optical = tf[cam.depth_optical_frame]
        view = optical[:3, 2]                       # optical Z = viewing direction
        headings[cam.name] = math.degrees(math.atan2(view[1], view[0]))
        if cam.external:
            assert abs(view[2]) < 1e-9              # front D455 is mounted level
            assert optical[2, 3] == pytest.approx(0.1945 + rover_geometry.BASE_LINK_HEIGHT)
        else:
            assert view[2] < 0.0                    # side cameras tilted down
            assert optical[2, 3] > 0.7              # on the post, well above the deck
    assert abs(headings['front']) < 5.0
    assert 60.0 < headings['left'] < 100.0
    assert -100.0 < headings['right'] < -60.0


def test_cameras_not_inside_rover_body(full_urdf):
    rig = camera_config.load(CONFIG)
    tf = rover_geometry.frame_transforms(full_urdf, rig.target_frame)
    for cam in rig.cameras:
        p = tf[cam.link_frame][:3, 3]
        for lo, hi in rover_geometry.ROVER_BOXES:
            assert not (np.all(p >= lo) and np.all(p <= hi)), (cam.name, lo, hi)
