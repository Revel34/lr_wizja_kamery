#!/usr/bin/env python3
"""Publishes synthetic D455-like point clouds for every camera in cameras.yaml.

Each camera is ray-cast (with the D455 field of view and the decimated resolution the real driver
produces) against a small static test scene around a stationary rover, so the whole pipeline
(TF, fusion, ground estimation, obstacle detection, clustering) can be checked without hardware.

Scene (base_link coordinates, ground at z = 0):
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from wizja_bringup import camera_config

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
    lo = np.asarray(lo)
    hi = np.asarray(hi)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / dirs
        t1 = (lo - origin) * inv
        t2 = (hi - origin) * inv
    t_near = np.nanmax(np.minimum(t1, t2), axis=1)
    t_far = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (t_far >= t_near) & (t_far > 0.0)
    return np.where(hit, np.where(t_near > 0.0, t_near, np.inf), np.inf)


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


def rpy_matrix(roll, pitch, yaw):
    return np.array(camera_config.rotation_matrix(roll, pitch, yaw))


class FakeCameras(Node):
    def __init__(self):
        super().__init__('fake_cameras')
        path = self.declare_parameter('cameras_config', '').value
        rate = self.declare_parameter('rate', 15.0).value
        width = self.declare_parameter('width', 320).value       # 640x480 with decimation 2
        height = self.declare_parameter('height', 240).value
        hfov = math.radians(self.declare_parameter('hfov_deg', 86.0).value)
        vfov = math.radians(self.declare_parameter('vfov_deg', 57.0).value)
        self._min_depth = self.declare_parameter('min_depth', 0.4).value
        self._max_depth = self.declare_parameter('max_depth', 10.0).value
        # Depth noise sigma = k * z^2 (D455: ~2% RMS at 4 m before filtering).
        self._noise_k = self.declare_parameter('noise_k', 0.0025).value
        self._dropout = self.declare_parameter('dropout', 0.05).value

        rig = camera_config.load(path)
        self._rng = np.random.default_rng(1)

        # Pixel rays in the optical frame (z forward, x right, y down), z component = 1
        # so the ray parameter t equals the depth value.
        fx = (width / 2.0) / math.tan(hfov / 2.0)
        fy = (height / 2.0) / math.tan(vfov / 2.0)
        u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
        rays_opt = np.stack([
            ((u - width / 2.0) / fx).ravel(),
            ((v - height / 2.0) / fy).ravel(),
            np.ones(width * height),
        ], axis=1)
        link_from_optical = rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2)

        self._cams = []
        for cam in rig.cameras:
            base_from_link = rpy_matrix(cam.roll, cam.pitch, cam.yaw)
            origin = np.array([cam.x, cam.y, cam.z])
            dirs = rays_opt @ (base_from_link @ link_from_optical).T
            depth = self._raycast(origin, dirs)
            valid = np.isfinite(depth) & (depth >= self._min_depth) & (depth <= self._max_depth)
            self._cams.append({
                'frame': cam.depth_optical_frame,
                'rays': rays_opt[valid].astype(np.float32),
                'depth': depth[valid].astype(np.float32),
                'pub': self.create_publisher(PointCloud2, cam.cloud_topic, qos_profile_sensor_data),
            })
            self.get_logger().info(
                f'{cam.node_name}: {int(valid.sum())} valid pixels -> {cam.cloud_topic} '
                f'[{cam.depth_optical_frame}]')

        self.create_timer(1.0 / max(1.0, rate), self._publish)

    def _raycast(self, origin, dirs):
        t = intersect_terrain(origin, dirs, self._min_depth, self._max_depth)
        for lo, hi in BOXES:
            t = np.minimum(t, intersect_box(origin, dirs, lo, hi))
        for cyl in CYLINDERS:
            t = np.minimum(t, intersect_cylinder(origin, dirs, *cyl))
        for sph in SPHERES:
            t = np.minimum(t, intersect_sphere(origin, dirs, *sph))
        return t

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for cam in self._cams:
            depth = cam['depth']
            noisy = depth + self._rng.normal(0.0, 1.0, depth.shape).astype(np.float32) * (
                self._noise_k * depth * depth)
            keep = self._rng.random(depth.shape) >= self._dropout
            points = cam['rays'][keep] * noisy[keep, None]
            header = Header(stamp=stamp, frame_id=cam['frame'])
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
