"""
Offline occupancy-grid test (no robot / no ROS runtime needed).

Usage
-----
  # Task 1 with a real scan captured via:  ros2 topic echo -n 1 /scan > scan.yaml
  python3 offline_test.py scan.yaml
  python3 offline_test.py scan.yaml --yaw-offset 1.5708     # if lidar is rotated

  # No scan file yet: synthetic room
  python3 offline_test.py
  python3 offline_test.py --move        # simulate Task 2 (drive 1 m, update every 0.1 m)

  Add --no-show to only save PNGs (e.g. over SSH).
"""

import argparse
import math
import random

try:                                   # installed as ROS package
    from occgrid_pkg.grid_core import OccupancyGrid, draw_grid, save_grid_image
except ImportError:                    # run directly from the source folder
    from grid_core import OccupancyGrid, draw_grid, save_grid_image


# ---------------------------------------------------------------------- #
# Load a scan saved with `ros2 topic echo -n 1 /scan > file`
# ---------------------------------------------------------------------- #
def load_scan_yaml(path):
    import yaml
    with open(path) as f:
        docs = [d for d in yaml.safe_load_all(f) if d]   # file ends with '---'
    msg = docs[0]
    # YAML parses '.inf' / '.nan' to float inf / nan automatically
    ranges = [float(r) for r in msg['ranges']]
    scan = {
        'ranges': ranges,
        'angle_min': float(msg['angle_min']),
        'angle_increment': float(msg['angle_increment']),
        'range_min': float(msg['range_min']),
        'range_max': float(msg['range_max']),
    }
    frame = msg.get('header', {}).get('frame_id', '?')
    n_valid = sum(1 for r in ranges if math.isfinite(r))
    print(f"Loaded scan: frame_id={frame}, {len(ranges)} beams ({n_valid} valid), "
          f"angle_min={scan['angle_min']:.3f}, inc={scan['angle_increment']:.5f}, "
          f"range=[{scan['range_min']}, {scan['range_max']}]")
    return scan


# ---------------------------------------------------------------------- #
# Synthetic scan: a rectangular room with a box, for testing without data
# ---------------------------------------------------------------------- #
def _segments():
    xmin, xmax, ymin, ymax = -1.5, 3.0, -1.2, 1.8          # room walls (m)
    bx, by, h = 1.8, 0.6, 0.2                             # box centre, half-size
    pts = [((xmin, ymin), (xmax, ymin)), ((xmax, ymin), (xmax, ymax)),
           ((xmax, ymax), (xmin, ymax)), ((xmin, ymax), (xmin, ymin)),
           ((bx - h, by - h), (bx + h, by - h)), ((bx + h, by - h), (bx + h, by + h)),
           ((bx + h, by + h), (bx - h, by + h)), ((bx - h, by + h), (bx - h, by - h))]
    return pts


def synthetic_scan(pose, n_beams=360, range_max=12.0, noise=0.01):
    x, y, th = pose
    inc = 2 * math.pi / n_beams
    ranges = []
    for i in range(n_beams):
        a = th - math.pi + i * inc
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (x1, y1), (x2, y2) in _segments():
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / den      # distance along beam
            s = ((x1 - x) * dy - (y1 - y) * dx) / den      # position on segment
            if t > 0 and 0 <= s <= 1:
                best = min(best, t)
        if best < range_max and random.random() > 0.02:    # 2% dropouts -> inf
            ranges.append(best + random.gauss(0, noise))
        else:
            ranges.append(math.inf)
    return {'ranges': ranges, 'angle_min': -math.pi, 'angle_increment': inc,
            'range_min': 0.15, 'range_max': range_max}


def apply(grid, scan, pose, yaw_offset):
    grid.update(scan['ranges'], scan['angle_min'], scan['angle_increment'],
                scan['range_min'], scan['range_max'], pose, yaw_offset)


# ---------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser()
    p.add_argument('scan_file', nargs='?', help='YAML from ros2 topic echo -n 1 /scan')
    p.add_argument('--yaw-offset', type=float, default=0.0,
                   help='lidar yaw relative to base_link (rad)')
    p.add_argument('--move', action='store_true',
                   help='simulate Task 2 with synthetic data')
    p.add_argument('--no-show', action='store_true')
    args = p.parse_args()

    if args.move:
        # Task 2 simulation: drive 1 m along +x, update every 0.1 m,
        # snapshots at 0.0 / 0.5 / 1.0 m
        grid = OccupancyGrid()
        snaps = []
        for k in range(11):
            pose = (0.1 * k, 0.0, 0.0)
            apply(grid, synthetic_scan(pose), pose, 0.0)
            if k in (0, 5, 10):
                name = f'sim_grid_{0.1 * k:.1f}m.png'
                save_grid_image(grid, name, pose, f'after {0.1 * k:.1f} m ({k + 1} updates)')
                snaps.append((grid.to_image().copy(), pose, name))
                print('saved', name)
        if not args.no_show:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
            tmp = OccupancyGrid()
            for ax, (img, pose, name) in zip(axes, snaps):
                ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
                          extent=tmp.extent(), interpolation='nearest')
                ax.plot(pose[0], pose[1], 'ro', markersize=4)
                ax.set_title(name)
            plt.tight_layout()
            plt.show()
        return

    grid = OccupancyGrid()
    pose = (0.0, 0.0, 0.0)
    scan = load_scan_yaml(args.scan_file) if args.scan_file else synthetic_scan(pose)
    apply(grid, scan, pose, args.yaw_offset)
    out = 'task1_grid.png'
    save_grid_image(grid, out, pose, 'Task 1: single scan')
    print('saved', out)

    if not args.no_show:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        draw_grid(ax, grid, pose, 'Task 1: single scan')
        plt.show()


if __name__ == '__main__':
    main()
