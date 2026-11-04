"""
HW9 calibration helper. It never moves the robot.

  ros2 run hw9pkg cone_calib --ros-args --params-file config/hw9_params.yaml

Once per second it prints:
- the HSV colour of the patch at the image centre and a suggested HSV box.
  Point the camera at a cone to measure its colour.
- what each configured colour detects, with its direction.
- the nearest lidar object straight ahead. With the bumper just touching a
  cone, this is `contact_range`.

It writes <output_dir>/calib_latest.png: the camera view next to each
colour's mask. Press Enter to keep a numbered copy. Ctrl-C quits.
"""

import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from hw9pkg.geometry import find_cluster
from hw9pkg.mission import MissionError, fmt_bearing
from hw9pkg.ros_robot import RosRobot
from hw9pkg.vision import (annotate, centre_hsv, detect_all, draw_colour,
                           min_blob_area, pixel_to_bearing, put_label,
                           suggest_box)


def mosaic(bgr, dets, masks, colours):
    h, w = bgr.shape[:2]
    view = annotate(bgr, dets)
    cv2.rectangle(view, (w // 2 - 9, h // 2 - 9), (w // 2 + 9, h // 2 + 9), (255, 255, 255), 1)
    tiles = [view]
    for c in colours:
        m = cv2.cvtColor(masks[c], cv2.COLOR_GRAY2BGR)
        put_label(m, f'{c} mask', (6, 18), draw_colour(c))
        tiles.append(m)
    return np.hstack(tiles)


def status(robot, fps):
    cfg = robot.cfg
    lines = []
    f = robot.latest_frame()
    if f is None:
        lines.append(f'camera: no images on {robot.image_topic} yet')
        img = None
    else:
        hue, sat, val = centre_hsv(f.bgr)
        lines.append(f'camera {f.width}x{f.height}, {fps:.1f} fps | centre patch H={hue} '
                     f'S={sat} V={val} -> suggested box {suggest_box(hue, sat, val)}')
        dets, masks = detect_all(f.bgr, robot.colour_table,
                                 min_blob_area(f.width, f.height, cfg.min_blob_fraction),
                                 want_masks=True)
        parts = []
        for c in cfg.colors:
            b = dets[c]
            if b is None:
                parts.append(f'{c}: -')
            else:
                rel = pixel_to_bearing(b.cx, f.width, math.radians(cfg.camera_hfov_deg))
                parts.append(f'{c}: {b.area} px, {fmt_bearing(rel)}')
        lines.append('  detections  ' + ' | '.join(parts))
        for i, a in enumerate(cfg.colors):
            for b in cfg.colors[i + 1:]:
                both = int(np.count_nonzero(masks[a] & masks[b]))
                if both > 50:
                    lines.append(f'  WARNING: {a} and {b} masks overlap on {both} px; '
                                 f'make their HSV boxes distinct')
        img = mosaic(f.bgr, dets, masks, cfg.colors)
    s = robot.latest_scan()
    if s is None or s.ranges.size == 0:
        lines.append(f'lidar: no scans on {robot.scan_topic} yet')
    else:
        cl = find_cluster(s.bearings, s.ranges, 0.0, math.radians(10))
        k = int(np.argmin(s.ranges))
        ahead = (f'{cl.range:.3f} m at {math.degrees(cl.bearing):+.1f} deg ({cl.count} pts)'
                 if cl else 'nothing')
        lines.append(f'  lidar: nearest within 10 deg of straight ahead: {ahead} | '
                     f'nearest anywhere: {s.ranges[k]:.2f} m at '
                     f'{math.degrees(s.bearings[k]):+.0f} deg')
    return '\n'.join(lines), img


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    robot = executor = None
    try:
        robot = RosRobot('cone_calib')
        executor = SingleThreadedExecutor()
        executor.add_node(robot)
        threading.Thread(target=executor.spin, daemon=True).start()
        out = robot.cfg.output_dir
        os.makedirs(out, exist_ok=True)

        save = threading.Event()

        def reader():
            for _ in sys.stdin:
                save.set()
        threading.Thread(target=reader, daemon=True).start()

        print(f'Colours: {", ".join(robot.cfg.colors)}. Writing {out}/calib_latest.png. '
              f'Enter = save a numbered snapshot, Ctrl-C = quit.', flush=True)
        n_saved, last_seq, last_t = 0, 0, time.monotonic()
        while True:
            time.sleep(1.0)
            f = robot.latest_frame()
            now = time.monotonic()
            seq = f.seq if f else 0
            fps = (seq - last_seq) / (now - last_t)
            last_seq, last_t = seq, now
            text, img = status(robot, fps)
            print('\n' + text, flush=True)
            if img is not None:
                cv2.imwrite(os.path.join(out, 'calib_latest.png'), img)
                if save.is_set():
                    save.clear()
                    n_saved += 1
                    path = os.path.join(out, f'calib_{n_saved:02d}.png')
                    cv2.imwrite(path, img)
                    print(f'  saved {path}', flush=True)
    except KeyboardInterrupt:
        pass
    except MissionError as e:
        print(f'\nERROR: {e}', flush=True)
    finally:
        if executor is not None:
            executor.shutdown()
        if robot is not None:
            robot.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
