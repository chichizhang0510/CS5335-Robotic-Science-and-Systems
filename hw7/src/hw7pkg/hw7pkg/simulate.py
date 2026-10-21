"""
Synthetic HW7 Task 1 (no robot / no ROS runtime needed).

Writes run1.npz and run2.npz in the same format as mapping_node, so
`landmarks` and `plan_path` can be tested before going to the lab:

  python3 simulate.py                 # -> ./hw7_output/run1.npz, run2.npz
  python3 simulate.py --out sim_output --length 3.5 --curve 0.3

The robot drives `length` metres (optionally along an arc), turns 180 deg,
and drives back. The grid is built from the *odometry* pose, which drifts
from the true pose (scale error, heading drift, imperfect turn), just like
on the real robot.
"""

import argparse
import math
import os
import random

import numpy as np

try:                                   # installed as ROS package
    from hw7pkg.grid_core import OccupancyGrid, save_grid_image
except ImportError:                    # run directly from the source folder
    from grid_core import OccupancyGrid, save_grid_image


LIDAR_OFFSET = math.pi / 2             # TurtleBot 4 lidar mount, as on the robot


def _box(cx, cy, hx, hy):
    a, b, c, d = (cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)
    return [(a, b), (b, c), (c, d), (d, a)]


def environment():
    """Line segments and circles (cx, cy, r) of a room with boxes and posts."""
    segs = []
    # room with an alcove on the top wall -> several distinct corners
    wall = [(-1.5, -1.6), (5.0, -1.6), (5.0, 2.0), (3.2, 2.0), (3.2, 2.6),
            (2.0, 2.6), (2.0, 2.0), (-1.5, 2.0), (-1.5, -1.6)]
    segs += list(zip(wall[:-1], wall[1:]))
    segs += _box(1.6, 0.9, 0.25, 0.15)      # boxes
    segs += _box(3.0, -1.0, 0.15, 0.15)
    segs += _box(0.5, -0.45, 0.10, 0.10)    # small box near the path (Task 3 obstacle)
    circles = [(0.8, -1.1, 0.04), (2.4, 1.3, 0.04), (4.2, 0.6, 0.04)]   # posts
    return segs, circles


def cast(x, y, a, segs, circles, range_max):
    dx, dy = math.cos(a), math.sin(a)
    best = math.inf
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        s = ((x1 - x) * dy - (y1 - y) * dx) / den
        if t > 0 and 0 <= s <= 1:
            best = min(best, t)
    for cx, cy, r in circles:
        fx, fy = x - cx, y - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0:
                best = min(best, t)
    return best if best < range_max else math.inf


def synthetic_scan(true_pose, env, n_beams=720, range_max=12.0, noise=0.01):
    x, y, th = true_pose
    inc = 2 * math.pi / n_beams
    ranges = []
    for i in range(n_beams):
        a = th + LIDAR_OFFSET - math.pi + i * inc
        r = cast(x, y, a, *env, range_max)
        if math.isfinite(r) and random.random() > 0.02:      # 2% dropouts -> inf
            ranges.append(r + random.gauss(0, noise))
        else:
            ranges.append(math.inf)
    return {'ranges': ranges, 'angle_min': -math.pi, 'angle_increment': inc,
            'range_min': 0.15, 'range_max': range_max}


def to_frame(origin, pose):
    """Express absolute `pose` in the frame whose origin is `origin`."""
    x0, y0, th0 = origin
    dx, dy = pose[0] - x0, pose[1] - y0
    c, s = math.cos(-th0), math.sin(-th0)
    return (c * dx - s * dy, s * dx + c * dy,
            math.atan2(math.sin(pose[2] - th0), math.cos(pose[2] - th0)))


def drive(grid, env, true, odom, length, curvature, step, rng):
    """Drive `length` m, updating the grid every `step` m. Returns final poses."""
    origin = tuple(odom)
    update_poses, trajectory = [], []
    n = int(round(length / step))
    for k in range(n + 1):
        if k:                              # move one step (true and odom differ)
            for p, scale, drift in ((true, 1.0, 0.0), (odom, rng['scale'], rng['drift'])):
                ds = step * scale
                p[2] += curvature * step / 2 + drift / 2
                p[0] += ds * math.cos(p[2])
                p[1] += ds * math.sin(p[2])
                p[2] += curvature * step / 2 + drift / 2
        local = to_frame(origin, odom)
        scan = synthetic_scan(true, env)
        grid.update(scan['ranges'], scan['angle_min'], scan['angle_increment'],
                    scan['range_min'], scan['range_max'], local, LIDAR_OFFSET)
        update_poses.append((*local, k * step))
        trajectory.append(local)
    return origin, np.array(update_poses), np.array(trajectory)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default='hw7_output')
    p.add_argument('--length', type=float, default=3.2, help='path length (m)')
    p.add_argument('--curve', type=float, default=0.0, help='path curvature (1/m)')
    p.add_argument('--seed', type=int, default=1)
    args = p.parse_args()
    random.seed(args.seed)
    os.makedirs(args.out, exist_ok=True)
    env = environment()
    noise = {'scale': 1.02, 'drift': math.radians(0.15)}   # per 0.1 m step

    true = [0.0, 0.0, 0.0]
    odom = [0.0, 0.0, 0.0]
    for run in ('run1', 'run2'):
        if run == 'run2':                  # turn 180: real turn is 3 deg short
            true[2] += math.pi - math.radians(3.0)
            odom[2] += math.pi - math.radians(1.0)
        grid = OccupancyGrid()
        origin, ups, traj = drive(grid, env, true, odom, args.length,
                                  args.curve if run == 'run1' else -args.curve,
                                  0.1, noise)
        path = os.path.join(args.out, f'{run}.npz')
        grid.save(path, odom_origin=np.array(origin), update_poses=ups,
                  trajectory=traj, laser_yaw_offset=LIDAR_OFFSET)
        end = math.hypot(*traj[-1][:2])
        save_grid_image(grid, os.path.join(args.out, f'{run}_final.png'), tuple(traj[-1]),
                        f'{run} (simulated): {args.length:.2f} m travelled, '
                        f'ends {end:.2f} m from start', trajectory=traj)
        print(f'saved {path} ({len(ups)} updates, ends {end:.2f} m from start)')


if __name__ == '__main__':
    main()
