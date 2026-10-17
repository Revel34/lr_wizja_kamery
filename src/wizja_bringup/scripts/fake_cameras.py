#!/usr/bin/env python3
"""Publishes synthetic D455-like point clouds for every camera in cameras.yaml.

Each camera is ray-cast (with the D455 field of view and the resolution the real driver produces) against a small static test scene around a stationary rover, so the whole pipeline
(TF, fusion, ground estimation, obstacle detection, clustering) can be checked without hardware.
Camera poses come from the rover URDF (wizja_description, built from cameras.yaml) and the rover's
own body (deck, rockers, wheels, post, mast) is part of the scene, so the cameras see the rover
exactly where the real ones would - which exercises the fusion node's self-filter.
Cameras driven by this project publish a PointCloud2 like realsense2_camera; `source: external`
cameras (the front D455 owned by the main rover stack) publish a 16UC1 depth image + CameraInfo,
like the main stack's driver, which exercises the fusion node's depth-image input.

Scene (base_footprint coordinates, ground at z = 0):
  box_front   0.4 x 0.4 x 0.40 m at x=2.5..2.9, y=0.1..0.5            -> obstacle
  box_left    0.6 x 0.4 x 0.60 m at x=0.5..1.1, y=2.3..2.7            -> obstacle
  pole_right  cylinder r=0.12 m, 1.0 m tall at (1.0, -2.5)            -> obstacle
  rock        sphere r=0.25 m sitting on the hill at (4.0, -3.0)      -> obstacle
  hill        smooth 0.5 m mound at (5.0, -3.0), max slope ~14 deg     -> ground, NOT an obstacle
  bump        0.3 x 0.3 x 0.08 m at x=2.0..2.3, y=-0.8..-0.5          -> below threshold, NOT an obstacle
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from wizja_bringup import camera_config, rover_geometry

HILL_CENTER = (5.0, -3.0)
HILL_HEIGHT = 0.5
HILL_SIGMA = 1.2

BOXES = [  # (min xyz, max xyz)
    ((2.5, 0.1, 0.0), (2.9, 0.5, 0.40)),
    ((0.5, 2.3, 0.0), (1.1, 2.7, 0.60)),
    ((2.0, -0.8, 0.0), (2.3, -0.5, 0.08)),
]
CYLINDERS = [  # (cx, cy, radius, z_min, z_max)
    (1.0, -2.5, 0.12, 0.0, 1.0),
]


def hill(x, y):
    dx = x - HILL_CENTER[0]
    dy = y - HILL_CENTER[1]
    return HILL_HEIGHT * np.exp(-(dx * dx + dy * dy) / (2.0 * HILL_SIGMA ** 2))


SPHERES = [  # (cx, cy, cz, radius)
    (4.0, -3.0, float(hill(4.0, -3.0)) + 0.15, 0.25),
]


def intersect_terrain(origin, dirs, t_min, t_max, step=0.02):
    """Ray march against z = hill(x, y); returns t (inf where nothing is hit)."""
    t_hit = np.full(dirs.shape[0], np.inf)
    active = np.ones(dirs.shape[0], dtype=bool)
    t_prev = t_min
    p = origin + t_prev * dirs
    f_prev = p[:, 2] - hill(p[:, 0], p[:, 1])
    t = t_min + step
    while t <= t_max and active.any():
        p = origin + t * dirs
        f = p[:, 2] - hill(p[:, 0], p[:, 1])
        crossed = active & (f <= 0.0)
        if crossed.any():
            # Linear interpolation of the zero crossing inside [t_prev, t].
            frac = f_prev[crossed] / (f_prev[crossed] - f[crossed])
            t_hit[crossed] = t_prev + frac * step
            active &= ~crossed
        f_prev = f
        t_prev = t
        t += step
    return t_hit


def intersect_box(origin, dirs, lo, hi):
    return rover_geometry.ray_box_t(origin, dirs, lo, hi)


def intersect_cylinder(origin, dirs, cx, cy, r, z_min, z_max):
    ox, oy = origin[0] - cx, origin[1] - cy
    a = dirs[:, 0] ** 2 + dirs[:, 1] ** 2
    b = 2.0 * (ox * dirs[:, 0] + oy * dirs[:, 1])
    c = ox * ox + oy * oy - r * r
    disc = b * b - 4.0 * a * c
    with np.errstate(invalid='ignore', divide='ignore'):
        t = (-b - np.sqrt(disc)) / (2.0 * a)
    z = origin[2] + t * dirs[:, 2]
    ok = (disc >= 0.0) & (t > 0.0) & (z >= z_min) & (z <= z_max)
    return np.where(ok, t, np.inf)


def intersect_sphere(origin, dirs, cx, cy, cz, r):
    oc = origin - np.array([cx, cy, cz])
    a = np.einsum('ij,ij->i', dirs, dirs)
    b = 2.0 * dirs @ oc
    c = oc @ oc - r * r
    disc = b * b - 4.0 * a * c
    with np.errstate(invalid='ignore'):
        t = (-b - np.sqrt(disc)) / (2.0 * a)
    return np.where((disc >= 0.0) & (t > 0.0), t, np.inf)


def pixel_rays(width, height, hfov, vfov):
    """Rays through pixel centres in the optical frame (z = 1, so ray t = depth) + intrinsics."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    fy = (height / 2.0) / math.tan(vfov / 2.0)
    # Pixel u covers [u, u+1); its centre u+0.5 is the principal ray when u+0.5 = width/2.
    cx, cy = width / 2.0 - 0.5, height / 2.0 - 0.5
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack([((u - cx) / fx).ravel(), ((v - cy) / fy).ravel(), np.ones(width * height)], axis=1)
    return rays, (fx, fy, cx, cy)


class FakeCameras(Node):
    def __init__(self):
        super().__init__('fake_cameras')
        path = self.declare_parameter('cameras_config', '').value
        rate = self.declare_parameter('rate', 15.0).value
        # Driver cameras (our realsense nodes): point cloud, 640x480 with decimation 2.
        width = self.declare_parameter('width', 320).value
        height = self.declare_parameter('height', 240).value
        hfov = math.radians(self.declare_parameter('hfov_deg', 86.0).value)
        vfov = math.radians(self.declare_parameter('vfov_deg', 57.0).value)
        # External cameras (main-stack driver): raw depth image, 640x480 like rs.launch.py.
        ext_width = self.declare_parameter('external_width', 640).value
        ext_height = self.declare_parameter('external_height', 480).value
        self._min_depth = self.declare_parameter('min_depth', 0.4).value
        self._max_depth = self.declare_parameter('max_depth', 10.0).value
        # Depth noise sigma = k * z^2 (D455: ~2% RMS at 4 m before filtering).
        self._noise_k = self.declare_parameter('noise_k', 0.0025).value
        self._dropout = self.declare_parameter('dropout', 0.05).value

        path = path or camera_config.default_config_path()
        rig = camera_config.load(path)
        # target_frame -> <camera>_depth_optical_frame, straight from the URDF that
        # robot_state_publisher publishes in sim_test.launch.py.
        transforms = rover_geometry.frame_transforms(
            rover_geometry.expand_urdf(path, include_rover=True, nominal_camera_frames=True),
            rig.target_frame)
        self._rng = np.random.default_rng(1)

        cloud_rays, _ = pixel_rays(width, height, hfov, vfov)
        image_rays, image_k = pixel_rays(ext_width, ext_height, hfov, vfov)

        self._cams = []
        for cam in rig.cameras:
            target_from_optical = transforms[cam.depth_optical_frame]
            origin = target_from_optical[:3, 3]
            rays = image_rays if cam.external else cloud_rays
            depth = self._raycast(origin, rays @ target_from_optical[:3, :3].T)
            valid = np.isfinite(depth) & (depth >= self._min_depth) & (depth <= self._max_depth)
            entry = {'frame': cam.depth_optical_frame, 'external': cam.external}
            if cam.external:
                entry['depth'] = np.where(valid, depth, 0.0).astype(np.float32)
                entry['shape'] = (ext_height, ext_width)
                entry['info'] = self._camera_info(ext_width, ext_height, image_k)
                entry['pub'] = self.create_publisher(Image, cam.depth_topic, qos_profile_sensor_data)
                entry['info_pub'] = self.create_publisher(
                    CameraInfo, cam.info_topic, qos_profile_sensor_data)
                target = f'{cam.depth_topic} (16UC1 {ext_width}x{ext_height}) + {cam.info_topic}'
            else:
                entry['rays'] = cloud_rays[valid].astype(np.float32)
                entry['depth'] = depth[valid].astype(np.float32)
                entry['pub'] = self.create_publisher(
                    PointCloud2, cam.cloud_topic, qos_profile_sensor_data)
                target = cam.cloud_topic
            self._cams.append(entry)
            self.get_logger().info(
                f'{cam.name}: {int(valid.sum())} valid pixels -> {target} [{cam.depth_optical_frame}]')

        self.create_timer(1.0 / max(1.0, rate), self._publish)

    @staticmethod
    def _camera_info(width, height, k):
        fx, fy, cx, cy = k
        info = CameraInfo()
        info.width, info.height = width, height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _raycast(self, origin, dirs):
        t = intersect_terrain(origin, dirs, self._min_depth, self._max_depth)
        for lo, hi in BOXES:
            t = np.minimum(t, intersect_box(origin, dirs, lo, hi))
        for cyl in CYLINDERS:
            t = np.minimum(t, intersect_cylinder(origin, dirs, *cyl))
        for sph in SPHERES:
            t = np.minimum(t, intersect_sphere(origin, dirs, *sph))
        return np.minimum(t, rover_geometry.first_rover_hit(origin, dirs))

    def _noisy(self, depth):
        noisy = depth + self._rng.normal(0.0, 1.0, depth.shape).astype(np.float32) * (
            self._noise_k * depth * depth)
        keep = self._rng.random(depth.shape) >= self._dropout
        return noisy, keep

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for cam in self._cams:
            header = Header(stamp=stamp, frame_id=cam['frame'])
            noisy, keep = self._noisy(cam['depth'])
            if cam['external']:
                mm = np.where(keep & (cam['depth'] > 0.0), np.clip(noisy * 1000.0, 1, 65535), 0)
                img = Image(header=header, height=cam['shape'][0], width=cam['shape'][1],
                            encoding='16UC1', is_bigendian=0, step=cam['shape'][1] * 2)
                img.data = mm.astype('<u2').tobytes()
                cam['info'].header = header
                cam['info_pub'].publish(cam['info'])
                cam['pub'].publish(img)
            else:
                points = cam['rays'][keep] * noisy[keep, None]
                cam['pub'].publish(point_cloud2.create_cloud_xyz32(
                    header, np.ascontiguousarray(points, dtype=np.float32)))


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameras()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
