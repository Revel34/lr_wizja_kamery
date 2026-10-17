"""Rover 3.0 geometry shared by the simulator (fake_cameras.py) and the coverage tool.

Camera poses are NOT duplicated here: they are read from the URDF produced by
wizja_description/urdf/wizja_rover.urdf.xacro (which itself reads cameras.yaml), so every tool sees
exactly the transforms robot_state_publisher publishes.
"""

import math
import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import numpy as np
from ament_index_python.packages import get_package_share_directory

# base_footprint -> base_link height (rover30_BASE_PLATFORM.xacro).
BASE_LINK_HEIGHT = 0.37956

_Z = BASE_LINK_HEIGHT

# Axis-aligned boxes approximating the rover body in base_footprint (ground at z = 0), measured
# from the rover30 meshes. Used for occlusion (what the cameras cannot see past) and as simulated
# self-returns. (min xyz, max xyz)
ROVER_BOXES: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = [
    ((-0.41, -0.217, _Z - 0.046), (0.40, 0.218, _Z + 0.18)),      # deck / electronics box
    ((-0.395, 0.16, _Z - 0.044), (0.515, 0.407, _Z + 0.126)),    # left rocker
    ((-0.395, -0.407, _Z - 0.044), (0.515, -0.16, _Z + 0.126)),  # right rocker
    ((0.441, 0.24, 0.075), (0.521, 0.416, _Z + 0.02)),           # left front steer
    ((0.441, -0.416, 0.075), (0.521, -0.24, _Z + 0.02)),         # right front steer
    ((-0.401, 0.24, 0.075), (-0.321, 0.416, _Z + 0.02)),         # left rear steer
    ((-0.401, -0.416, 0.075), (-0.321, -0.24, _Z + 0.02)),       # right rear steer
    ((0.351, 0.312, 0.0), (0.611, 0.453, 0.246)),                # left front wheel
    ((0.351, -0.454, 0.0), (0.611, -0.304, 0.246)),              # right front wheel
    ((-0.491, 0.312, 0.0), (-0.231, 0.453, 0.246)),              # left rear wheel
    ((-0.491, -0.454, 0.0), (-0.231, -0.304, 0.246)),            # right rear wheel
    ((-0.51, 0.03, _Z + 0.18), (-0.30, 0.25, _Z + 0.715)),       # rear mast
    ((0.31, -0.02, _Z + 0.18), (0.35, 0.02, _Z + 0.38)),         # camera post
]


def expand_urdf(cameras_config: str, include_rover: bool = True,
                nominal_camera_frames: bool = True) -> str:
    """Run xacro on wizja_rover.urdf.xacro and return the URDF XML string."""
    import xacro  # imported lazily: only needed by tools that build the model

    path = os.path.join(
        get_package_share_directory('wizja_description'), 'urdf', 'wizja_rover.urdf.xacro')
    doc = xacro.process_file(path, mappings={
        'cameras_config': cameras_config,
        'include_rover': 'true' if include_rover else 'false',
        'nominal_camera_frames': 'true' if nominal_camera_frames else 'false',
    })
    return doc.toprettyxml(indent='  ')


def _origin_matrix(joint) -> np.ndarray:
    from wizja_bringup.camera_config import rotation_matrix

    origin = joint.find('origin')
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    m = np.eye(4)
    m[:3, :3] = np.array(rotation_matrix(*rpy))
    m[:3, 3] = xyz
    return m


def frame_transforms(urdf_xml: str, target_frame: str) -> Dict[str, np.ndarray]:
    """4x4 transforms target_frame -> every link of the URDF (movable joints at position 0)."""
    root = ET.fromstring(urdf_xml)
    joints = {}
    for joint in root.findall('joint'):
        joints[joint.find('child').get('link')] = (joint.find('parent').get('link'), joint)

    cache: Dict[str, np.ndarray] = {}

    def from_root(link: str) -> np.ndarray:
        if link in cache:
            return cache[link]
        if link not in joints:
            cache[link] = np.eye(4)
        else:
            parent, joint = joints[link]
            cache[link] = from_root(parent) @ _origin_matrix(joint)
        return cache[link]

    links = [link.get('name') for link in root.findall('link')]
    if target_frame not in links:
        raise ValueError(f'frame {target_frame!r} not in the URDF')
    target_inv = np.linalg.inv(from_root(target_frame))
    return {name: target_inv @ from_root(name) for name in links}


def ray_box_t(origins: np.ndarray, dirs: np.ndarray, lo, hi) -> np.ndarray:
    """Ray parameter t of the first hit of rays origin + t * dir with a box (inf = miss).

    origins: (3,) or (N, 3), dirs: (N, 3). Rays starting inside the box hit at t = 0.
    """
    lo = np.asarray(lo, dtype=float)
    hi = np.asarray(hi, dtype=float)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / dirs
        t1 = (lo - origins) * inv
        t2 = (hi - origins) * inv
    t_near = np.nanmax(np.minimum(t1, t2), axis=1)
    t_far = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (t_far >= np.maximum(t_near, 0.0))
    return np.where(hit, np.maximum(t_near, 0.0), np.inf)


def first_rover_hit(origins: np.ndarray, dirs: np.ndarray) -> np.ndarray:
    """Smallest t at which each ray hits any rover box (inf = no hit)."""
    t = np.full(dirs.shape[0], np.inf)
    for lo, hi in ROVER_BOXES:
        t = np.minimum(t, ray_box_t(origins, dirs, lo, hi))
    return t


def footprint_contains(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """True where (x, y) lies under any rover box (for drawing the rover on maps)."""
    inside = np.zeros(np.shape(x), dtype=bool)
    for lo, hi in ROVER_BOXES:
        inside |= (x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1])
    return inside


def yaw_deg(matrix: np.ndarray) -> float:
    """Heading of a frame's X axis in its parent frame, degrees CCW from the parent X axis."""
    return math.degrees(math.atan2(matrix[1, 0], matrix[0, 0]))
