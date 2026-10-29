#!/usr/bin/env python3
"""Simulated D455 cameras (and, with drive:=true, a keyboard-drivable rover) for testing without hardware.

Each camera is ray-cast (with the D455 field of view) against a small test scene, so the whole
pipeline (TF, fusion, ground estimation, obstacle detection, clustering) can be checked without
hardware. Camera poses come from the rover URDF (wizja_description, CAMERA POSITIONS block) and the
rover's own body (deck, rockers, wheels, post, mast) is part of the scene, so the cameras see the
rover exactly where the real ones would - which exercises the fusion node's self-filter.
Cameras driven by this project (all three by default) publish a PointCloud2 like realsense2_camera;
`source: external` cameras (owned by another process) publish a 16UC1 depth image + CameraInfo like
that process's driver would, which exercises the fusion node's depth-image input.

drive:=false (default): the rover stands at the origin; the scene is rendered once.
drive:=true: the rover follows /cmd_vel (e.g. teleop_twist_keyboard) with Rover 3.0's 4-wheel
  independent steering: forward/back, sideways and turning on the spot. Purely kinematic - no physics,
  no collisions, the rover stays at z = 0 (it does not climb the hill). Publishes odom ->
  base_footprint TF, /odom, /wizja/joint_states (steer angles + wheel spin for the RViz model) and
  /sim/scene (the true objects as markers). Cameras are re-rendered from the current pose every frame,
  at a lower resolution (drive_resolution_scale) to keep the frame rate up.

Scene (odom = world frame; the rover starts at the origin facing +x, ground at z = 0):
  box_front   0.4 x 0.4 x 0.40 m at x=2.5..2.9, y=0.1..0.5            -> obstacle
  box_left    0.6 x 0.4 x 0.60 m at x=0.5..1.1, y=2.3..2.7            -> obstacle
  pole_right  cylinder r=0.12 m, 1.0 m tall at (1.0, -2.5)            -> obstacle
  rock        sphere r=0.25 m sitting on the hill at (4.0, -3.0)      -> obstacle
  hill        smooth 0.5 m mound at (5.0, -3.0), max slope ~14 deg     -> ground, NOT an obstacle
  bump        0.3 x 0.3 x 0.08 m at x=2.0..2.3, y=-0.8..-0.5          -> below threshold, NOT an obstacle
  further than 9 m from the start (out of range there, to drive to): a big box ahead (10, -1), a rock
  ahead-left (9.5, 3), a low wall at the far left, a pole behind (-9.5, 0.5), a box behind-right.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from wizja_bringup import camera_config, rover_geometry, sim_kinematics

HILL_CENTER = (5.0, -3.0)
HILL_HEIGHT = 0.5
HILL_SIGMA = 1.2
HILL_RADIUS = 3.5 * HILL_SIGMA  # beyond this the hill is < 1 mm high: treated as flat ground

BOXES = [  # (min xyz, max xyz)
    ((2.5, 0.1, 0.0), (2.9, 0.5, 0.40)),
    ((0.5, 2.3, 0.0), (1.1, 2.7, 0.60)),
    ((2.0, -0.8, 0.0), (2.3, -0.5, 0.08)),       # bump (below the obstacle threshold)
    ((9.5, -1.5, 0.0), (10.5, -0.5, 0.80)),      # far: big box ahead
    ((6.0, 8.0, 0.0), (9.0, 8.3, 0.50)),         # far: low wall ahead-left
    ((-7.0, -7.5, 0.0), (-6.4, -6.9, 0.50)),     # far: box behind-right
]
CYLINDERS = [  # (cx, cy, radius, z_min, z_max)
    (1.0, -2.5, 0.12, 0.0, 1.0),
    (-9.5, 0.5, 0.10, 0.0, 1.2),                 # far: pole behind
]


def hill(x, y):
    dx = x - HILL_CENTER[0]
    dy = y - HILL_CENTER[1]
    return HILL_HEIGHT * np.exp(-(dx * dx + dy * dy) / (2.0 * HILL_SIGMA ** 2))


SPHERES = [  # (cx, cy, cz, radius)
    (4.0, -3.0, float(hill(4.0, -3.0)) + 0.15, 0.25),
    (9.5, 3.0, 0.20, 0.35),                      # far: rock ahead-left
]


# Largest slope of the hill (max |grad| of the Gaussian): used to bound how fast a ray can approach it.
HILL_MAX_SLOPE = HILL_HEIGHT / HILL_SIGMA * math.exp(-0.5)


def intersect_terrain(origin, dirs, t_min, t_max, min_step=0.01):
    """First hit of rays with the ground z = hill(x, y); t = inf where nothing is hit.

    Flat ground is intersected analytically. Only rays crossing the hill's bounding cylinder below the
    hill top are marched, with conservative (Lipschitz) steps: a ray's height above the ground can
    shrink by at most (|dz| + HILL_MAX_SLOPE * |dxy|) per unit t, so stepping by
    height / that rate never jumps over the surface - unlike fixed sampling, rays skimming the
    hill's shoulder are not missed. A ray stops only once it has crossed the surface, and the crossing
    is interpolated linearly between the last two points (error well below 1 cm). min_step keeps rays
    that skim just above the hill moving.
    """
    oz, dz = origin[2], dirs[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        t_ground = np.where(dz < 0.0, -oz / dz, np.inf)
        t_below_top = np.where(dz < 0.0, (HILL_HEIGHT - oz) / dz, np.where(oz <= HILL_HEIGHT, 0.0, np.inf))
    t_hit = np.where((t_ground >= t_min) & (t_ground <= t_max), t_ground, np.inf)

    # Interval [t0, t1] inside the vertical cylinder around the hill.
    ox, oy = origin[0] - HILL_CENTER[0], origin[1] - HILL_CENTER[1]
    a = dirs[:, 0] ** 2 + dirs[:, 1] ** 2
    b = 2.0 * (ox * dirs[:, 0] + oy * dirs[:, 1])
    c = ox * ox + oy * oy - HILL_RADIUS ** 2
    disc = b * b - 4.0 * a * c
    with np.errstate(divide='ignore', invalid='ignore'):
        sq = np.sqrt(np.maximum(disc, 0.0))
        t0 = (-b - sq) / (2.0 * a)
        t1 = (-b + sq) / (2.0 * a)
    ta = np.maximum.reduce([t0, t_below_top, np.full_like(t0, t_min)])
    tb = np.minimum.reduce([t1, t_ground, np.full_like(t1, t_max)])
    idx = np.flatnonzero((disc > 0.0) & (a > 1e-12) & (tb > ta))
    if idx.size == 0:
        return t_hit

    d = dirs[idx]
    rate = np.abs(d[:, 2]) + HILL_MAX_SLOPE * np.sqrt(d[:, 0] ** 2 + d[:, 1] ** 2)
    t = ta[idx].copy()
    end = tb[idx]
    prev_t = t.copy()
    prev_f = np.full(t.shape, np.inf)
    active = np.arange(idx.size)
    while active.size:
        ta_ = t[active]
        p = origin[None, :] + ta_[:, None] * d[active]
        f = p[:, 2] - hill(p[:, 0], p[:, 1])          # height of the ray above the ground
        hit = f <= 0.0
        if hit.any():
            rows = active[hit]
            fp, fc_ = prev_f[rows], f[hit]
            th = t[rows].copy()               # ray starts below the surface: hit at its first point
            crossed = np.isfinite(fp)
            th[crossed] = prev_t[rows][crossed] + (t[rows][crossed] - prev_t[rows][crossed]) * (
                fp[crossed] / (fp[crossed] - fc_[crossed]))
            t_hit[idx[rows]] = th
        prev_t[active] = ta_
        prev_f[active] = f
        step = np.maximum(f / rate[active], min_step)
        t[active] = ta_ + step
        keep = ~hit & (t[active] <= end[active])
        active = active[keep]
    return t_hit


def intersect_box(origin, inv_dirs, lo, hi):
    """Slab test with precomputed 1/dirs (shared by all boxes of a frame); inf = miss."""
    t1 = (np.asarray(lo) - origin) * inv_dirs
    t2 = (np.asarray(hi) - origin) * inv_dirs
    t_near = np.minimum(t1, t2).max(axis=1)
    t_far = np.maximum(t1, t2).min(axis=1)
    hit = t_far >= np.maximum(t_near, 0.0)
    return np.where(hit, np.maximum(t_near, 0.0), np.inf)


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


def yaw_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class FakeCameras(Node):
    def __init__(self):
        super().__init__('fake_cameras')
        path = self.declare_parameter('cameras_config', '').value
        rate = self.declare_parameter('rate', 15.0).value
        self._drive = self.declare_parameter('drive', False).value
        # Driver cameras (our realsense nodes): point cloud, 640x480 with decimation 2.
        width = self.declare_parameter('width', 320).value
        height = self.declare_parameter('height', 240).value
        hfov = math.radians(self.declare_parameter('hfov_deg', 86.0).value)
        vfov = math.radians(self.declare_parameter('vfov_deg', 57.0).value)
        # External cameras (main-stack driver): raw depth image, 640x480 like rs.launch.py.
        ext_width = self.declare_parameter('external_width', 640).value
        ext_height = self.declare_parameter('external_height', 480).value
        # Drive mode renders every frame: scale all resolutions down to keep the frame rate up.
        scale = self.declare_parameter('drive_resolution_scale', 0.5).value if self._drive else 1.0
        width, height = max(8, int(width * scale)), max(6, int(height * scale))
        ext_width, ext_height = max(8, int(ext_width * scale)), max(6, int(ext_height * scale))
        self._min_depth = self.declare_parameter('min_depth', 0.4).value
        self._max_depth = self.declare_parameter('max_depth', 10.0).value
        # Depth noise sigma = k * z^2 (D455: ~2% RMS at 4 m before filtering).
        self._noise_k = self.declare_parameter('noise_k', 0.0025).value
        self._dropout = self.declare_parameter('dropout', 0.05).value

        path = path or camera_config.default_config_path()
        rig = camera_config.load(path)
        self._target_frame = rig.target_frame
        # target_frame -> <camera>_depth_optical_frame, straight from the URDF that
        # robot_state_publisher publishes in sim_test.launch.py.
        urdf = rover_geometry.expand_urdf(include_rover=True, nominal_camera_frames=True)
        rover_geometry.check_cameras_in_urdf(rig, urdf, include_external=True)
        transforms = rover_geometry.frame_transforms(urdf, rig.target_frame)
        self._rng = np.random.default_rng(1)

        cloud_rays, _ = pixel_rays(width, height, hfov, vfov)
        image_rays, image_k = pixel_rays(ext_width, ext_height, hfov, vfov)

        self._cams = []
        for cam in rig.cameras:
            target_from_optical = transforms[cam.depth_optical_frame]
            rays = image_rays if cam.external else cloud_rays
            pixels = None
            if cam.external and cam.decimation > 1:
                # The fusion node only reads every decimation-th pixel in both directions: render
                # just those, the rest of the published image stays 0 (= no depth).
                v, u = np.divmod(np.arange(ext_width * ext_height), ext_width)
                pixels = np.flatnonzero((u % cam.decimation == 0) & (v % cam.decimation == 0))
                rays = rays[pixels]
            entry = {
                'name': cam.name,
                'frame': cam.depth_optical_frame,
                'external': cam.external,
                'rays': rays.astype(np.float32),
                # Camera origin / ray directions in the rover frame (base_footprint), computed once.
                'origin': target_from_optical[:3, 3].copy(),
                'dirs': rays @ target_from_optical[:3, :3].T,
            }
            # The rover body moves with the cameras: its occlusion is ray-cast once and cached.
            entry['t_rover'] = rover_geometry.first_rover_hit(entry['origin'], entry['dirs'])
            if cam.external:
                entry['shape'] = (ext_height, ext_width)
                entry['pixels'] = pixels
                entry['info'] = self._camera_info(ext_width, ext_height, image_k)
                entry['pub'] = self.create_publisher(Image, cam.depth_topic, qos_profile_sensor_data)
                entry['info_pub'] = self.create_publisher(
                    CameraInfo, cam.info_topic, qos_profile_sensor_data)
                target = f'{cam.depth_topic} (16UC1 {ext_width}x{ext_height}) + {cam.info_topic}'
            else:
                entry['pub'] = self.create_publisher(
                    PointCloud2, cam.cloud_topic, qos_profile_sensor_data)
                target = f'{cam.cloud_topic} ({width}x{height} rays)'
            self._cams.append(entry)
            self.get_logger().info(f'{cam.name} -> {target} [{cam.depth_optical_frame}]')

        if self._drive:
            self._setup_drive(transforms_base_link=rover_geometry.frame_transforms(urdf, 'base_link'))
        else:
            # Static rover at the origin: render once, re-publish with fresh noise.
            for cam in self._cams:
                cam['depth'] = self._render(cam, np.zeros(3), 0.0)
        self.create_timer(1.0 / max(1.0, rate), self._publish)

    # ------------------------------------------------------------------ drive mode
    def _setup_drive(self, transforms_base_link):
        self._pose = sim_kinematics.RoverPose()
        # Steer joint positions (x, y) in base_link: the swerve kinematics' wheel lever arms.
        self._wheels = {
            w: tuple(transforms_base_link[f'{w}_steer_link'][:2, 3])
            for w in ('left_front', 'right_front', 'left_rear', 'right_rear')
        }
        self._steer = {w: 0.0 for w in self._wheels}
        self._spin = {w: 0.0 for w in self._wheels}
        self._last_step = self._now_s()
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._joint_pub = self.create_publisher(JointState, '/wizja/joint_states', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._scene_pub = self.create_publisher(MarkerArray, '/sim/scene', latched)
        self._scene_pub.publish(self._scene_markers())
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(1.0 / 30.0, self._step)
        self.get_logger().info(
            'Drive mode: steer with /cmd_vel, e.g. `ros2 run teleop_twist_keyboard '
            'teleop_twist_keyboard` (Shift+J/L = sideways). Kinematic only: no collisions, no climbing.')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg):
        self._pose.command(msg.linear.x, msg.linear.y, msg.angular.z, self._now_s())

    def _step(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        dt = min(0.2, t - self._last_step)
        self._last_step = t
        p = self._pose
        p.step(t, dt)
        self._steer, speeds = sim_kinematics.swerve_wheels(p.vx, p.vy, p.wz, self._wheels, self._steer)
        for w, v in speeds.items():
            self._spin[w] = math.remainder(
                self._spin[w] + v / sim_kinematics.WHEEL_RADIUS * dt, 2.0 * math.pi)
        stamp = now.to_msg()
        self._publish_pose(stamp)

        names, positions = [], []
        for w in self._wheels:
            names += [f'{w}_steer_joint', f'{w}_wheel_joint']
            positions += [self._steer[w], self._spin[w]]
        js = JointState()
        js.header.stamp = stamp
        js.name = names
        js.position = positions
        self._joint_pub.publish(js)

    def _publish_pose(self, stamp):
        p = self._pose
        q = (0.0, 0.0, math.sin(p.yaw / 2.0), math.cos(p.yaw / 2.0))
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = self._target_frame
        tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
        tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = self._target_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = p.x, p.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2], q[3]
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = p.vx, p.vy
        odom.twist.twist.angular.z = p.wz
        self._odom_pub.publish(odom)

    def _scene_markers(self):
        markers = MarkerArray()

        def marker(mid, mtype, xyz, scale, rgba, text=''):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'sim_scene'
            m.id = mid
            m.type = mtype
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.text = text
            return m

        obstacle, low = (1.0, 0.55, 0.1, 0.35), (0.6, 0.6, 0.6, 0.35)
        mid = 0
        for lo, hi in BOXES:
            size = tuple(h - l for l, h in zip(lo, hi))
            centre = tuple((l + h) / 2.0 for l, h in zip(lo, hi))
            markers.markers.append(marker(mid, Marker.CUBE, centre, size, low if size[2] < 0.15 else obstacle))
            mid += 1
        for cx, cy, r, z0, z1 in CYLINDERS:
            markers.markers.append(marker(mid, Marker.CYLINDER, (cx, cy, (z0 + z1) / 2.0), (2 * r, 2 * r, z1 - z0), obstacle))
            mid += 1
        for cx, cy, cz, r in SPHERES:
            markers.markers.append(marker(mid, Marker.SPHERE, (cx, cy, cz), (2 * r,) * 3, obstacle))
            mid += 1
        # The hill as a flat translucent disc (2 sigma radius) at half its height.
        markers.markers.append(marker(
            mid, Marker.CYLINDER, (HILL_CENTER[0], HILL_CENTER[1], HILL_HEIGHT / 2.0),
            (4.0 * HILL_SIGMA, 4.0 * HILL_SIGMA, HILL_HEIGHT), (0.3, 0.7, 0.3, 0.2)))
        return markers

    # ------------------------------------------------------------------ rendering
    def _render(self, cam, position, yaw):
        """Depth per ray (0 = no return) for the rover at (position, yaw) in odom."""
        rot = yaw_rotation(yaw)
        origin = rot @ cam['origin'] + position
        dirs = cam['dirs'] @ rot.T
        # Rays are scaled so t = depth >= distance: objects farther than max_depth can be skipped.
        reach = self._max_depth

        def near(cx, cy, radius):
            return math.hypot(cx - origin[0], cy - origin[1]) - radius <= reach

        t = intersect_terrain(origin, dirs, self._min_depth, self._max_depth)
        inv_dirs = None
        for lo, hi in BOXES:
            centre = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)
            if near(*centre, math.hypot(hi[0] - lo[0], hi[1] - lo[1]) / 2.0):
                if inv_dirs is None:
                    # Zero components give +/-inf, which the slab test handles (no NaN: the
                    # rays' z component is never 0 together with origin exactly on a slab plane).
                    with np.errstate(divide='ignore'):
                        inv_dirs = 1.0 / dirs
                with np.errstate(invalid='ignore'):
                    t = np.minimum(t, intersect_box(origin, inv_dirs, lo, hi))
        for cyl in CYLINDERS:
            if near(cyl[0], cyl[1], cyl[2]):
                t = np.minimum(t, intersect_cylinder(origin, dirs, *cyl))
        for sph in SPHERES:
            if near(sph[0], sph[1], sph[3]):
                t = np.minimum(t, intersect_sphere(origin, dirs, *sph))
        t = np.minimum(t, cam['t_rover'])
        valid = np.isfinite(t) & (t >= self._min_depth) & (t <= self._max_depth)
        return np.where(valid, t, 0.0).astype(np.float32)

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

    def _noisy(self, depth):
        noisy = depth + self._rng.normal(0.0, 1.0, depth.shape).astype(np.float32) * (
            self._noise_k * depth * depth)
        keep = (self._rng.random(depth.shape) >= self._dropout) & (depth > 0.0)
        return noisy, keep

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        if self._drive:
            # TF with the same stamp as the images, so RViz can place them in odom immediately.
            self._publish_pose(stamp)
            position = np.array([self._pose.x, self._pose.y, 0.0])
        for cam in self._cams:
            depth = self._render(cam, position, self._pose.yaw) if self._drive else cam['depth']
            header = Header(stamp=stamp, frame_id=cam['frame'])
            noisy, keep = self._noisy(depth)
            if cam['external']:
                mm = np.where(keep, np.clip(noisy * 1000.0, 1, 65535), 0).astype('<u2')
                if cam['pixels'] is not None:
                    full = np.zeros(cam['shape'][0] * cam['shape'][1], dtype='<u2')
                    full[cam['pixels']] = mm
                    mm = full
                img = Image(header=header, height=cam['shape'][0], width=cam['shape'][1],
                            encoding='16UC1', is_bigendian=0, step=cam['shape'][1] * 2)
                img.data = mm.tobytes()
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
