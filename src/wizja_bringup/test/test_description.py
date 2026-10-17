import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from wizja_bringup import camera_config, rover_geometry

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'cameras.yaml')


@pytest.fixture(scope='module')
def full_urdf():
    return rover_geometry.expand_urdf(include_rover=True, nominal_camera_frames=True)


@pytest.fixture(scope='module')
def cameras_only_urdf():
    return rover_geometry.expand_urdf(include_rover=False, nominal_camera_frames=False)


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


def test_every_yaml_camera_is_placed_in_the_urdf(full_urdf, cameras_only_urdf):
    rig = camera_config.load(CONFIG)
    rover_geometry.check_cameras_in_urdf(rig, full_urdf, include_external=True)
    rover_geometry.check_cameras_in_urdf(rig, cameras_only_urdf, include_external=False)
    tf = rover_geometry.frame_transforms(full_urdf, 'base_link')
    for cam in rig.cameras:
        assert cam.depth_optical_frame in tf


def test_missing_camera_is_reported(full_urdf, tmp_path):
    path = tmp_path / 'cams.yaml'
    path.write_text('cameras:\n  rear:\n    serial_no: \'1\'\n')
    with pytest.raises(RuntimeError, match='camera_rear_link'):
        rover_geometry.check_cameras_in_urdf(camera_config.load(str(path)), full_urdf, True)


# base_link -> camera_link poses (xyz, rpy) the rover is currently set up with. The front one is the
# rover30 camera_joint (tripod screw 0.42, 0, 0.18) + the D455 screw -> camera_link offset; the side
# ones are the post design. Update together with the CAMERA POSITIONS block when cameras move.
EXPECTED_LINK_POSES = {
    'camera_link': ((0.43115, 0.0475, 0.1945), (0.0, 0.0, 0.0)),
    'camera_left_link': ((0.245, 0.106, 0.395), (0.0, 0.35, 1.3963)),
    'camera_right_link': ((0.338, -0.090, 0.395), (0.0, 0.35, -1.3963)),
}


def test_camera_link_poses(full_urdf):
    tf = rover_geometry.frame_transforms(full_urdf, 'base_link')
    for frame, (xyz, rpy) in EXPECTED_LINK_POSES.items():
        assert tf[frame][:3, 3] == pytest.approx(xyz, abs=1e-4), frame
        expected = np.array(camera_config.rotation_matrix(*rpy))
        assert np.allclose(tf[frame][:3, :3], expected, atol=1e-6), frame


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
