"""
HW9: calibration and checks to run before cone_finder. Nothing here moves the robot.

  ros2 run hw9pkg calibrate tune orange            live sliders for one colour's HSV range;
                                                   s saves it to hsv_ranges.yaml, q quits
  ros2 run hw9pkg calibrate tune orange --image hw9_output/snap_01.png
                                                   same, on a saved picture (no robot needed)
  ros2 run hw9pkg calibrate snap                   save a camera frame, print what it detects
                                                   and the HSV at the image centre
  ros2 run hw9pkg calibrate look                   camera angle + lidar distance of every
                                                   visible cone, without moving
  ros2 run hw9pkg calibrate lidar                  lidar distance straight ahead: put the cone
                                                   touching the bumper -> touch_range
  ros2 run hw9pkg calibrate detect a.png b.png     detection on saved pictures (no robot needed)

Run from the hw9 folder so hsv_ranges.yaml and hw9_output/ land there.
ROS parameters of the shared robot interface (topics, laser_yaw_offset, ...)
can be added after --ros-args, as for cone_finder.
"""

import argparse
import math
import os
import statistics
import sys

import cv2
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args

from hw9pkg.color_detect import (DEFAULT_HSV_FILE, annotate, color_mask, detect, largest_blob,
                                 load_hsv, median_hsv, save_hsv, to_hsv)
from hw9pkg.lidar_objects import find_object, scan_points
from hw9pkg.robot import RobotIO, shutdown, spin_in_background

TRACKBARS = ['H lo', 'S lo', 'V lo', 'H hi', 'S hi', 'V hi']   # same order as the HSV tuples
TRACKBAR_MAX = [179, 255, 255, 179, 255, 255]


def pick_colors(arg, table):
    if not arg:
        return sorted(table)
    colors = [c.strip().lower() for c in arg.split(',') if c.strip()]
    unknown = [c for c in colors if c not in table]
    if unknown:
        sys.exit(f'No HSV range for {unknown}; known colours: {sorted(table)}')
    return colors


def next_snap_path(out_dir):
    k = 1
    while os.path.exists(os.path.join(out_dir, f'snap_{k:02d}.png')):
        k += 1
    return os.path.join(out_dir, f'snap_{k:02d}.png')


# ---------------------------------------------------------------------- #
def tune(args, node=None):
    table, _ = load_hsv(args.hsv_file)
    color = args.color.lower()
    rng = table.get(color, (0, 100, 100, 179, 255, 255))
    still = None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            sys.exit(f'Cannot read {args.image}')

    win = f'tune {color}'
    cv2.namedWindow(win)
    for name, val, top in zip(TRACKBARS, rng, TRACKBAR_MAX):
        cv2.createTrackbar(name, win, int(val), top, lambda _v: None)
    view = {'frame': None, 'scale': 1}

    def on_mouse(event, x, y, _flags, _param):
        frame = view['frame']
        if event != cv2.EVENT_LBUTTONDOWN or frame is None:
            return
        u, v = x // view['scale'], y // view['scale']
        if u < frame.shape[1] and v < frame.shape[0]:
            print(f'  pixel ({u}, {v}): H, S, V = {median_hsv(frame, u, v)}')

    cv2.setMouseCallback(win, on_mouse)
    print(f'Tuning {color}. Move the sliders until only the cone is white in the right-hand mask.')
    print('Click the picture to print the HSV there. Keys: s = save, q = quit.')
    print('Hue wraps when H lo > H hi (red: H lo about 170, H hi about 6).')

    while True:
        frame = still if still is not None else node.latest_frame()
        if frame is None:
            frame = np.zeros((240, 320, 3), np.uint8)
        rng = [cv2.getTrackbarPos(n, win) for n in TRACKBARS]
        mask = color_mask(to_hsv(frame), rng)
        min_area = args.min_area_frac * frame.shape[0] * frame.shape[1]
        blob = largest_blob(mask, color, min_area)
        both = np.hstack([annotate(frame, {color: blob} if blob else {}),
                          cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        scale = max(1, 400 // frame.shape[0])
        if scale > 1:
            both = cv2.resize(both, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        view.update(frame=frame, scale=scale)
        cv2.imshow(win, both)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            save_hsv(args.hsv_file, color, rng)
            print(f'Saved {color}: {rng} -> {os.path.abspath(args.hsv_file)}')
    cv2.destroyAllWindows()


def snap(args, node):
    table, source = load_hsv(args.hsv_file)
    colors = pick_colors(args.colors, table)
    frame = node.frame()
    rows, cols = frame.shape[:2]
    path = next_snap_path(args.out)
    cv2.imwrite(path, frame)
    blobs = detect(frame, colors, table, args.min_area_frac)
    cv2.imwrite(path.replace('.png', '_detected.png'), annotate(frame, blobs))
    print(f'Saved {path} ({cols} x {rows}); HSV ranges from {source}')
    print(f'H, S, V at the image centre: {median_hsv(frame, cols // 2, rows // 2)}'
          '   (put a cone in the middle of the view)')
    for color in colors:
        b = blobs.get(color)
        if b is None:
            print(f'  {color:7s} not seen')
        else:
            print(f'  {color:7s} column {b.u:5.0f} -> {math.degrees(node.camera_bearing(b.u, cols)):+6.1f} deg,'
                  f' area {b.area:6.0f} px')


def look(args, node):
    table, _ = load_hsv(args.hsv_file)
    colors = pick_colors(args.colors, table)
    frame = node.frame()
    cols = frame.shape[1]
    scans = [node.points() for _ in range(3)]
    blobs = detect(frame, colors, table, args.min_area_frac)
    window = math.radians(args.window)
    print('colour   camera angle | lidar angle, distance, width   (+ = left)')
    for color in colors:
        b = blobs.get(color)
        if b is None:
            print(f'  {color:7s} not seen by the camera')
            continue
        cam = node.camera_bearing(b.u, cols)
        objs = [find_object(pts, cam, window, args.max_range, args.max_width, step) for pts, step in scans]
        objs = [o for o in objs if o is not None]
        if not objs:
            print(f'  {color:7s} {math.degrees(cam):+6.1f} deg   | NO lidar object within '
                  f'+-{args.window:.0f} deg and {args.max_range} m')
            continue
        print(f'  {color:7s} {math.degrees(cam):+6.1f} deg   | '
              f'{math.degrees(statistics.median(o.bearing for o in objs)):+6.1f} deg, '
              f'{statistics.median(o.range for o in objs):.3f} m, '
              f'{statistics.median(o.width for o in objs) * 100:.0f} cm')
    cv2.imwrite(os.path.join(args.out, 'look.png'), annotate(frame, blobs))
    print('Camera and lidar angles should agree within a few degrees (README: "Camera angle check").')


def lidar(args, node):
    found, body = [], set()
    for _ in range(5):
        pts, step = node.points()
        obj = find_object(pts, 0.0, math.radians(15), 1.5, 0.6, step)
        if obj is not None:
            found.append(obj)
        raw = scan_points(node._scan[0], node.yaw_offset, 0.0)
        body.update(round(math.degrees(a)) for a, r in raw if r < node.min_valid)
    if body:
        print(f'Rays shorter than min_valid_range ({node.min_valid} m), ignored as the robot body, '
              f'at about {sorted(body)[:12]} deg')
    if not found:
        print('Nothing within 1.5 m straight ahead (+-15 deg). Is laser_yaw_offset right?')
        return
    r = statistics.median(o.range for o in found)
    b = statistics.median(o.bearing for o in found)
    w = statistics.median(o.width for o in found)
    print(f'Object straight ahead: {r:.3f} m at {math.degrees(b):+.1f} deg, about {w * 100:.0f} cm wide')
    print(f'If the cone is touching the front bumper, run cone_finder with:  -p touch_range:={r:.3f}')


def detect_files(args):
    table, source = load_hsv(args.hsv_file)
    colors = pick_colors(args.colors, table)
    print(f'HSV ranges from {source}')
    for path in args.images:
        img = cv2.imread(path)
        if img is None:
            print(f'{path}: cannot read')
            continue
        blobs = detect(img, colors, table, args.min_area_frac)
        out = os.path.join(args.out, os.path.splitext(os.path.basename(path))[0] + '_detected.png')
        cv2.imwrite(out, annotate(img, blobs))
        found = ', '.join(f'{c} at ({b.u:.0f}, {b.v:.0f}) area {b.area:.0f}' for c, b in blobs.items())
        print(f'{path}: {found or "nothing"} -> {out}')


# ---------------------------------------------------------------------- #
def main():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--hsv-file', default=DEFAULT_HSV_FILE)
    common.add_argument('--out', default='hw9_output')
    common.add_argument('--colors', default='', help='comma list; default all known colours')
    common.add_argument('--min-area-frac', type=float, default=0.001)

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('tune', parents=[common], help='HSV sliders for one colour')
    p.add_argument('color')
    p.add_argument('--image', help='tune on a saved picture instead of the live camera')
    sub.add_parser('snap', parents=[common], help='save a frame and show detections')
    p = sub.add_parser('look', parents=[common], help='camera angle + lidar distance per cone')
    p.add_argument('--window', type=float, default=12.0, help='lidar search +- deg')
    p.add_argument('--max-range', type=float, default=3.0)
    p.add_argument('--max-width', type=float, default=0.45)
    sub.add_parser('lidar', parents=[common], help='lidar distance straight ahead')
    p = sub.add_parser('detect', parents=[common], help='detect cones in saved pictures')
    p.add_argument('images', nargs='+')

    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    os.makedirs(args.out, exist_ok=True)

    if args.command == 'detect':
        detect_files(args)
        return
    if args.command == 'tune' and args.image:
        tune(args)
        return

    rclpy.init(args=sys.argv)
    node = RobotIO('hw9_calibrate')
    executor = spin_in_background(node)
    try:
        node.wait_ready(camera=args.command != 'lidar')
        {'tune': tune, 'snap': snap, 'look': look, 'lidar': lidar}[args.command](args, node)
    except KeyboardInterrupt:
        pass
    except (TimeoutError, RuntimeError) as e:
        print(f'Error: {e}')
    finally:
        shutdown(node, executor)


if __name__ == '__main__':
    main()
