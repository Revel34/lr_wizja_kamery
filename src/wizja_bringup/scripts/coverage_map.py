#!/usr/bin/env python3
"""Prints an ASCII top-down map of what the cameras in cameras.yaml can see on the rover.

Use it to choose / check the mounting positions, yaw, pitch and post height:

    ros2 run wizja_bringup coverage_map.py                       # installed cameras.yaml
    ros2 run wizja_bringup coverage_map.py --config my.yaml --height 0.3 --range 6

Camera poses come from the rover URDF (wizja_description, built from cameras.yaml), so they are
exactly what robot_state_publisher publishes. A cell is "seen" when a point at --height above the
ground in that cell lies inside a camera's depth frustum (FOV and depth range) AND the line of
sight is not blocked by the rover body (deck, rockers, wheels, mast, post - rover_geometry.py).
Legend: ' ' unseen, '1'/'2'/'3' number of cameras seeing it, '#' rover.
"""

import argparse
import math

import numpy as np

from wizja_bringup import camera_config, rover_geometry


def visible(points, target_from_optical, hfov, vfov, min_depth, max_depth, occlusion=True):
    """Boolean mask of target-frame points the camera can see."""
    rot = target_from_optical[:3, :3]
    origin = target_from_optical[:3, 3]
    local = (points - origin) @ rot  # target -> optical (rot^T applied)
    z = local[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        mask = ((z >= min_depth) & (z <= max_depth) &
                (np.abs(local[:, 0] / z) <= math.tan(hfov / 2)) &
                (np.abs(local[:, 1] / z) <= math.tan(vfov / 2)))
    if occlusion and mask.any():
        # Segment camera -> point blocked by the rover before reaching the point?
        idx = np.flatnonzero(mask)
        t_hit = rover_geometry.first_rover_hit(origin, points[idx] - origin)
        mask[idx[t_hit < 0.999]] = False
    return mask


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--config', default='', help='cameras.yaml (default: installed one)')
    parser.add_argument('--height', type=float, default=0.0, help='height above ground to test [m]')
    parser.add_argument('--range', type=float, default=5.0, help='half-size of the map [m]')
    parser.add_argument('--cell', type=float, default=0.2, help='map cell size [m]')
    parser.add_argument('--hfov', type=float, default=86.0, help='horizontal FOV [deg] (D455: 86)')
    parser.add_argument('--vfov', type=float, default=57.0, help='vertical FOV [deg] (D455: 57)')
    parser.add_argument('--min-depth', type=float, default=0.4)
    parser.add_argument('--max-depth', type=float, default=8.0)
    parser.add_argument('--no-occlusion', action='store_true', help='ignore the rover body')
    args = parser.parse_args()

    path = args.config or camera_config.default_config_path()
    rig = camera_config.load(path)
    transforms = rover_geometry.frame_transforms(
        rover_geometry.expand_urdf(path, include_rover=True, nominal_camera_frames=True),
        rig.target_frame)
    cams = [(cam, transforms[cam.depth_optical_frame]) for cam in rig.cameras]
    hfov, vfov = math.radians(args.hfov), math.radians(args.vfov)
    occl = not args.no_occlusion

    def seen_by(points):
        return [visible(points, t, hfov, vfov, args.min_depth, args.max_depth, occl) for _, t in cams]

    n = int(round(2 * args.range / args.cell))
    centers = (np.arange(n) + 0.5) * args.cell - args.range
    xs, ys = np.meshgrid(centers, centers, indexing='ij')
    points = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, args.height)], axis=1)
    count = np.sum(seen_by(points), axis=0).reshape(n, n)
    rover = rover_geometry.footprint_contains(xs, ys)

    print(f'Coverage at {args.height:.2f} m above ground, cell {args.cell} m, X forward (up), '
          f'Y left (left), rover occlusion {"on" if occl else "off"}')
    for ix in range(n - 1, -1, -1):          # rows: +X at the top
        row = []
        for iy in range(n - 1, -1, -1):      # columns: +Y on the left
            if rover[ix, iy]:
                row.append('#')
            else:
                c = count[ix, iy]
                row.append(' ' if c == 0 else str(min(c, 9)))
        print('|' + ''.join(row) + '|')

    print('\nCameras (in %s):' % rig.target_frame)
    fine = np.linspace(0.0, args.max_depth, int(args.max_depth / 0.02) + 1)
    for cam, t in cams:
        heading = math.degrees(math.atan2(t[1, 2], t[0, 2]))  # optical Z axis = viewing direction
        # Nearest ground (at --height) the camera sees straight along its viewing direction.
        dirs = np.array([math.cos(math.radians(heading)), math.sin(math.radians(heading))])
        line = np.stack([t[0, 3] + fine * dirs[0], t[1, 3] + fine * dirs[1],
                         np.full(fine.size, args.height)], axis=1)
        vis = visible(line, t, hfov, vfov, args.min_depth, args.max_depth, occl)
        near = f'{fine[np.argmax(vis)]:.2f} m' if vis.any() else 'never'
        print(f'  {cam.name:6s} at ({t[0, 3]:+.2f}, {t[1, 3]:+.2f}, {t[2, 3]:.2f}) m, '
              f'heading {heading:+5.0f} deg, ground visible from {near} straight ahead')

    # Angular coverage on a ring around the rover.
    ring_r = min(args.range, 3.0)
    angles = np.radians(np.arange(0, 360, 1.0))
    ring = np.stack([ring_r * np.cos(angles), ring_r * np.sin(angles),
                     np.full(angles.size, args.height)], axis=1)
    seen = np.any(seen_by(ring), axis=0)
    print(f'\nAt {ring_r:.1f} m from {rig.target_frame}: {seen.mean() * 100:.0f}% of directions covered.')
    blind = ~seen
    if blind.any() and not blind.all():
        shift = int(np.argmax(seen))  # start scanning at a seen direction so sectors don't wrap
        start = None
        for k in np.roll(np.arange(angles.size), -shift).tolist() + [shift]:
            if blind[k] and start is None:
                start = k
            elif not blind[k] and start is not None:
                width = (k - start) % angles.size
                mid = (start + width / 2.0) % 360
                mid = mid - 360 if mid > 180 else mid
                print(f'  blind sector: {width:3d} deg wide, centred at {mid:+.0f} deg '
                      f'(0 = forward, +90 = left)')
                start = None


if __name__ == '__main__':
    main()
