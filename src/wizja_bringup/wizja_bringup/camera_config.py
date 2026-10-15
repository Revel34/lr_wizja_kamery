"""Loads config/cameras.yaml, shared by all wizja launch files and the fake camera simulator."""

import math
import os
from dataclasses import dataclass
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass
class CameraConfig:
    name: str           # short name, e.g. 'front'
    serial_no: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @property
    def node_name(self) -> str:
        """realsense2_camera node name and camera_name (frame prefix)."""
        return f'camera_{self.name}'

    @property
    def link_frame(self) -> str:
        return f'{self.node_name}_link'

    @property
    def depth_optical_frame(self) -> str:
        return f'{self.node_name}_depth_optical_frame'

    @property
    def cloud_topic(self) -> str:
        # realsense2_camera >= 4.51 publishes on ~/depth/color/points (node-name relative).
        return f'/{self.node_name}/depth/color/points'


@dataclass
class RigConfig:
    target_frame: str
    cameras: List[CameraConfig]
    realsense: Dict


def default_config_path() -> str:
    return os.path.join(get_package_share_directory('wizja_bringup'), 'config', 'cameras.yaml')


def load(path: str = '') -> RigConfig:
    path = path or default_config_path()
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    cameras_data = data.get('cameras') or {}
    if not cameras_data:
        raise ValueError(f'{path}: no cameras defined')

    cameras = []
    for name, entry in cameras_data.items():
        entry = entry or {}
        pose = entry.get('pose') or {}
        cameras.append(CameraConfig(
            name=str(name),
            # YAML turns an unquoted serial into an int; keep it a string either way.
            serial_no=str(entry.get('serial_no', '') or ''),
            x=float(pose.get('x', 0.0)),
            y=float(pose.get('y', 0.0)),
            z=float(pose.get('z', 0.0)),
            roll=float(pose.get('roll', 0.0)),
            pitch=float(pose.get('pitch', 0.0)),
            yaw=float(pose.get('yaw', 0.0)),
        ))

    return RigConfig(
        target_frame=str(data.get('target_frame', 'base_link')),
        cameras=cameras,
        realsense=dict(data.get('realsense') or {}),
    )


def fusion_parameters(rig: RigConfig) -> Dict:
    """Parameter overrides for the cloud_fusion node derived from the camera list."""
    params = {
        'target_frame': rig.target_frame,
        'camera_names': [c.name for c in rig.cameras],
    }
    for cam in rig.cameras:
        params[f'cameras.{cam.name}.topic'] = cam.cloud_topic
    return params


def rotation_matrix(roll: float, pitch: float, yaw: float):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll), the URDF / tf2 setRPY convention (row-major lists)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
