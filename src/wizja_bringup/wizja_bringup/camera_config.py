"""Loads config/cameras.yaml (which cameras exist and how to read them), shared by all wizja launch
files and tools. Camera POSITIONS are not here: they live in wizja_description's URDF."""

import math
import os
from dataclasses import dataclass
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory


SOURCES = ('driver', 'external')


@dataclass
class CameraConfig:
    name: str           # short name, e.g. 'front'
    serial_no: str
    source: str = 'driver'      # 'driver': we run realsense2_camera; 'external': another stack does
    frame_prefix: str = ''      # frame names <prefix>_link, ... (default camera_<name>)
    depth_topic: str = ''       # external only: depth image (16UC1 mm or 32FC1 m)
    info_topic: str = ''        # external only: CameraInfo of that depth image
    decimation: int = 1         # external only: use every n-th pixel in both directions

    @property
    def external(self) -> bool:
        return self.source == 'external'

    @property
    def node_name(self) -> str:
        """realsense2_camera node name and camera_name (frame prefix) for driver cameras."""
        return self.frame_prefix or f'camera_{self.name}'

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
    target_frame: str   # frame of the fused cloud / outputs (base_footprint: ground at z = 0)
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
        source = str(entry.get('source', 'driver'))
        if source not in SOURCES:
            raise ValueError(f'{path}: camera {name}: source must be one of {SOURCES}')
        if source == 'external' and not (entry.get('depth_topic') and entry.get('info_topic')):
            raise ValueError(f'{path}: external camera {name} needs depth_topic and info_topic')
        cameras.append(CameraConfig(
            name=str(name),
            # YAML turns an unquoted serial into an int; keep it a string either way.
            serial_no=str(entry.get('serial_no', '') or ''),
            source=source,
            frame_prefix=str(entry.get('frame_prefix', '') or ''),
            depth_topic=str(entry.get('depth_topic', '') or ''),
            info_topic=str(entry.get('info_topic', '') or ''),
            decimation=max(1, int(entry.get('decimation', 1))),
        ))

    return RigConfig(
        target_frame=str(data.get('target_frame', 'base_footprint')),
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
        prefix = f'cameras.{cam.name}.'
        if cam.external:
            params[prefix + 'type'] = 'depth_image'
            params[prefix + 'topic'] = cam.depth_topic
            params[prefix + 'info_topic'] = cam.info_topic
            params[prefix + 'decimation'] = cam.decimation
        else:
            params[prefix + 'type'] = 'cloud'
            params[prefix + 'topic'] = cam.cloud_topic
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
