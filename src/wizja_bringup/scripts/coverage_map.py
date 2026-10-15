#!/usr/bin/env python3
"""Prints an ASCII top-down map of what the cameras in cameras.yaml can see.

Use it to choose the mounting positions / yaw / pitch before drilling holes:

    ros2 run wizja_bringup coverage_map.py                       # installed config
    ros2 run wizja_bringup coverage_map.py --config my.yaml --height 0.3 --range 6

A cell is "seen" when a point at --height above the ground in that cell lies inside a camera's
depth frustum (horizontal/vertical FOV and depth range). Occlusion by the rover body is ignored.
Legend: ' ' unseen, '1'/'2'/'3' number of cameras seeing it, '#' rover (self-filter box).
"""

import argparse
import math

import numpy as np

from wizja_bringup import camera_config

LINK_FROM_OPTICAL = np.array(camera_config.rotation_matrix(-math.pi / 2, 0.0, -math.pi / 2))


def visible(points, cam, hfov, vfov, min_depth, max_depth):
    """Boolean mask of base_link points inside the camera's frustum."""
    rot = np.array(camera_config.rotation_matrix(cam.roll, cam.pitch, cam.yaw)) @ LINK_FROM_OPTICAL
    local = (points - np.array([cam.x, cam.y, cam.z])) @ rot  # base -> optical (rot^T applied)
    z = local[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        return ((z >= min_depth) & (z <= max_depth) &
                (np.abs(local[:, 0] / z) <= math.tan(hfov / 2)) &
                (np.abs(local[:, 1] / z) <= math.tan(vfov / 2)))


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
    parser.add_argument('--rover', type=float, nargs=4, default=[-0.7, 0.7, -0.65, 0.65],
                        metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'), help='rover footprint [m]')
    args = parser.parse_args()

    rig = camera_config.load(args.config)
    n = int(round(2 * args.range / args.cell))
    centers = (np.arange(n) + 0.5) * args.cell - args.range
    xs, ys = np.meshgrid(centers, centers, indexing='ij')
    points = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, args.height)], axis=1)

    count = np.zeros(points.shape[0], dtype=int)
    for cam in rig.cameras:
        count += visible(points, cam, math.radians(args.hfov), math.radians(args.vfov),
                         args.min_depth, args.max_depth)
    count = count.reshape(n, n)

    x0, x1, y0, y1 = args.rover
    print(f'Coverage at {args.height:.2f} m above ground, cell {args.cell} m, '
          f'X forward (up), Y left (left)')
    for ix in range(n - 1, -1, -1):          # rows: +X at the top
        row = []
        for iy in range(n - 1, -1, -1):      # columns: +Y on the left
            x, y = centers[ix], centers[iy]
            if x0 <= x <= x1 and y0 <= y <= y1:
                row.append('#')
            else:
                c = count[ix, iy]
                row.append(' ' if c == 0 else str(min(c, 9)))
        print('|' + ''.join(row) + '|')

    # Angular coverage in a ring around the rover (ignores the near field).
    ring_r = min(args.range, 3.0)
    angles = np.radians(np.arange(0, 360, 1.0))
    ring = np.stack([ring_r * np.cos(angles), ring_r * np.sin(angles), np.full(angles.size, args.height)], axis=1)
    seen = np.zeros(angles.size, dtype=bool)
    for cam in rig.cameras:
        seen |= visible(ring, cam, math.radians(args.hfov), math.radians(args.vfov),
                        args.min_depth, args.max_depth)
    print(f'\nAt {ring_r:.1f} m from base_link: {seen.mean() * 100:.0f}% of directions covered.')
    gaps = []
    start = None
    # Report contiguous blind sectors (angles measured CCW from forward).
    blind = ~seen
    if blind.any() and not blind.all():
        shift = int(np.argmax(seen))  # start scanning at a seen direction so sectors don't wrap
        order = np.roll(np.arange(angles.size), -shift)
        for k in order:
            if blind[k] and start is None:
                start = k
            if not blind[k] and start is not None:
                gaps.append((start, (k - 1) % angles.size))
                start = None
        if start is not None:
            gaps.append((start, order[-1]))
        for a, b in gaps:
            width = (b - a) % angles.size + 1
            mid = (a + width / 2.0) % 360
            mid = mid - 360 if mid > 180 else mid
            print(f'  blind sector: {width:3d} deg wide, centred at {mid:+.0f} deg '
                  f'(0 = forward, +90 = left)')


if __name__ == '__main__':
    main()
