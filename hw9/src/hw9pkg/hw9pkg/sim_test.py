"""
Offline end-to-end test of the HW9 mission. Needs no ROS and no robot.

A simulated TurtleBot 4 sits in a 6 m x 6 m room with three cones in a
triangle and an orange poster on one wall as a distraction.

- A pinhole camera renders the cones, so the real colour detector runs on
  every image.
- A 2D lidar sees each cone as a circle at lidar height.
- Odometry has a gyro scale error.
- Camera and lidar data arrive with realistic delays.

The test runs Task 1, then drives to the cones in the given order. It checks
the true bumper-to-cone gap at each stop and that nothing was ever touched.

  ros2 run hw9pkg sim_test
  ros2 run hw9pkg sim_test --order blue orange green blue --hfov-error 25 --gyro-error 0.03
"""

import argparse
import math
import sys

import cv2
import numpy as np

from hw9pkg.geometry import scan_to_polar
from hw9pkg.mission import ConeHunter, Frame, MissionConfig, Scan
from hw9pkg.vision import DEFAULT_HSV, detect_all, min_blob_area, parse_boxes

ROBOT_R = 0.17                 # Create 3 radius (m)
CAM_X, CAM_H = 0.08, 0.24      # camera: ahead of base centre, height
LIDAR_X, LIDAR_H = -0.04, 0.19
LIDAR_YAW = math.pi / 2        # lidar mounted +90 deg, as on the TB4
CONE_H, R_BOT, R_TOP = 0.30, 0.075, 0.02   # 12" cone body
BASE_R, BASE_H = 0.09, 0.03    # base plate (treated as round)
ROOM = 3.0                     # walls at +-3 m
IMG = 250                      # TB4 preview size
CMD_LATENCY = 0.05

SIM_BGR = {'orange': (0, 110, 250), 'green': (50, 170, 30), 'blue': (190, 80, 10),
           'red': (30, 20, 200), 'yellow': (0, 210, 230), 'purple': (150, 40, 110),
           'pink': (170, 110, 250)}


def cone_r(h):
    return R_BOT + (R_TOP - R_BOT) * (h - BASE_H) / (CONE_H - BASE_H)


# Lidar range to a cone when the bumper just touches its base, head on.
CONTACT_RANGE = ROBOT_R + BASE_R - LIDAR_X - cone_r(LIDAR_H)


class SimRobot:
    def __init__(self, cfg, cones, start=(0.0, 0.0, 0.0), hfov_deg=42.0,
                 gyro_scale=1.0, cam_hz=15.0, cam_latency=0.15,
                 scan_hz=7.5, scan_latency=0.08, lidar_blind=False, seed=1):
        self.cfg = cfg
        self.table = {c: parse_boxes(DEFAULT_HSV[c], c) for c in cfg.colors}
        self.cones = cones                        # [(colour, x, y)]
        self.posters = [('orange', ROOM - 0.01, -1.2, 0.25)]   # (colour, x, y, half width)
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.x, self.y, self.yaw = start          # true pose
        self.ox, self.oy, self.oyaw = start       # odometry
        self.gyro_scale = gyro_scale
        self.lidar_blind = lidar_blind            # cones below the lidar plane
        self.v = self.w = 0.0
        self.cmds = []                            # (apply_time, v, w)
        self.v_cmd = self.w_cmd = 0.0
        self.f = (IMG / 2) / math.tan(math.radians(hfov_deg) / 2)
        self.cam_dt, self.cam_lat = 1.0 / cam_hz, cam_latency
        self.scan_dt, self.scan_lat = 1.0 / scan_hz, scan_latency
        self.next_cam = self.next_scan = 0.0
        self.pending = []                         # (deliver_time, kind, payload)
        self.frame = self.scan = None
        self.frames = []
        self.seq_f = self.seq_s = 0
        self.bump_count = 0
        self.min_gap = math.inf

    # ------------------------------------------------- robot interface
    def now(self):
        return self.t

    def pose(self):
        return (self.ox, self.oy, self.oyaw)

    def latest_frame(self):
        return self.frame

    def latest_scan(self):
        return self.scan

    def pop_frames(self):
        out, self.frames = self.frames, []
        return out

    def send_cmd(self, v, w):
        self.cmds.append((self.t + CMD_LATENCY, float(v), float(w)))

    def stop(self):
        self.send_cmd(0.0, 0.0)

    def sleep(self, dt):
        end = self.t + dt
        while self.t < end - 1e-9:
            self._step(min(0.005, end - self.t))

    # ------------------------------------------------------ simulation
    def gap_to(self, colour):
        for c, cx, cy in self.cones:
            if c == colour:
                return math.hypot(self.x - cx, self.y - cy) - ROBOT_R - BASE_R
        raise KeyError(colour)

    def _step(self, dt):
        while self.cmds and self.cmds[0][0] <= self.t:
            _, self.v_cmd, self.w_cmd = self.cmds.pop(0)
        self.v += clip(self.v_cmd - self.v, -0.5 * dt, 0.5 * dt)
        self.w += clip(self.w_cmd - self.w, -2.0 * dt, 2.0 * dt)
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        gaps = [math.hypot(nx - cx, ny - cy) - ROBOT_R - BASE_R for _, cx, cy in self.cones]
        self.min_gap = min(self.min_gap, min(gaps))
        if min(gaps) < 0.0:
            self.bump_count += 1              # blocked by the cone
            self.v = 0.0
        else:
            self.x, self.y = nx, ny
            self.ox += self.v * math.cos(self.oyaw) * dt
            self.oy += self.v * math.sin(self.oyaw) * dt
        self.yaw += self.w * dt
        self.oyaw += self.w * self.gyro_scale * dt
        self.t += dt

        if self.t >= self.next_cam:
            self.next_cam += self.cam_dt
            self.pending.append((self.t + self.cam_lat, 'frame', (self._render(), self.pose())))
        if self.t >= self.next_scan:
            self.next_scan += self.scan_dt
            self.pending.append((self.t + self.scan_lat, 'scan', (self._lidar(), self.pose())))
        due = [p for p in self.pending if p[0] <= self.t]
        if due:
            self.pending = [p for p in self.pending if p[0] > self.t]
            for _, kind, (data, pose) in sorted(due, key=lambda p: p[0]):
                if kind == 'scan':
                    self.seq_s += 1
                    b, r = scan_to_polar(data, -math.pi, 2 * math.pi / len(data), 0.15, 12.0,
                                         self.cfg.laser_yaw_offset, self.cfg.min_valid_range)
                    self.scan = Scan(self.seq_s, self.t, b, r, pose, self.cfg.laser_x_offset)
                else:
                    self.seq_f += 1
                    dets = detect_all(data, self.table,
                                      min_blob_area(IMG, IMG, self.cfg.min_blob_fraction))
                    self.frame = Frame(self.seq_f, self.t, data, dets, pose, self.scan)
                    self.frames.append(self.frame)

    def _render(self):
        img = np.empty((IMG, IMG, 3), np.uint8)
        img[:IMG // 2] = (190, 195, 200)
        img[IMG // 2:] = (110, 110, 110)
        cx0 = self.x + CAM_X * math.cos(self.yaw)
        cy0 = self.y + CAM_X * math.sin(self.yaw)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        items = []
        for colour, px, py in self.cones:
            items.append(('cone', colour, px, py, 0.0))
        for colour, px, py, hw in self.posters:
            items.append(('poster', colour, px, py, hw))
        drawn = []
        for kind, colour, px, py, hw in items:
            dx, dy = px - cx0, py - cy0
            z = dx * c + dy * s
            lat = -dx * s + dy * c
            if z > 0.05:
                drawn.append((z, kind, colour, lat, hw))
        f, half = self.f, IMG / 2
        for z, kind, colour, lat, hw in sorted(drawn, reverse=True):
            u = half - f * lat / z

            def row(h):
                return half + f * (CAM_H - h) / z

            def quad(h0, r0, h1, r1, col):
                pts = np.array([[u - f * r0 / z, row(h0)], [u + f * r0 / z, row(h0)],
                                [u + f * r1 / z, row(h1)], [u - f * r1 / z, row(h1)]])
                pts = np.clip(pts, -4000, 4000) * 16
                cv2.fillConvexPoly(img, pts.astype(np.int32), col, cv2.LINE_AA, 4)

            col = SIM_BGR[colour]
            if kind == 'poster':
                quad(0.9, hw, 1.3, hw, col)
                continue
            quad(0.0, BASE_R, BASE_H, BASE_R, tuple(int(0.8 * v) for v in col))
            quad(BASE_H, R_BOT, CONE_H, R_TOP, col)
            quad(0.14, cone_r(0.14), 0.19, cone_r(0.19), (235, 235, 235))   # reflective band
        noise = self.rng.normal(0, 4, img.shape)
        return np.clip(img + noise, 0, 255).astype(np.uint8)

    def _lidar(self, n=720):
        laser = -math.pi + 2 * math.pi / n * np.arange(n)
        a = self.yaw + laser + LIDAR_YAW
        ox = self.x + LIDAR_X * math.cos(self.yaw)
        oy = self.y + LIDAR_X * math.sin(self.yaw)
        dx, dy = np.cos(a), np.sin(a)
        with np.errstate(divide='ignore'):
            tx = np.where(dx > 0, (ROOM - ox) / dx, (-ROOM - ox) / dx)
            ty = np.where(dy > 0, (ROOM - oy) / dy, (-ROOM - oy) / dy)
        r = np.minimum(np.abs(tx), np.abs(ty))
        rc = cone_r(LIDAR_H)
        for _, cx, cy in ([] if self.lidar_blind else self.cones):
            b = dx * (ox - cx) + dy * (oy - cy)
            cc = (ox - cx) ** 2 + (oy - cy) ** 2 - rc * rc
            disc = b * b - cc
            hit = disc >= 0
            t = -b - np.sqrt(np.where(hit, disc, 0))
            r = np.where(hit & (t > 0), np.minimum(r, t), r)
        r = r + self.rng.normal(0, 0.004, n)
        r[self.rng.random(n) < 0.03] = np.inf          # dropouts
        return r


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--colors', nargs=3, default=['orange', 'green', 'blue'])
    ap.add_argument('--order', nargs='+', default=['orange', 'blue', 'green', 'orange'])
    ap.add_argument('--radius', type=float, default=0.95,
                    help='distance from the robot to each cone (m)')
    ap.add_argument('--start-yaw', type=float, default=20.0, help='deg')
    ap.add_argument('--true-hfov', type=float, default=42.0, help='simulated camera FOV (deg)')
    ap.add_argument('--hfov-error', type=float, default=0.0,
                    help='configured minus true camera FOV (deg)')
    ap.add_argument('--gyro-error', type=float, default=0.02, help='odometry yaw scale error')
    ap.add_argument('--cam-hz', type=float, default=15.0, help='camera frame rate')
    ap.add_argument('--cam-latency', type=float, default=0.15, help='camera delay (s)')
    ap.add_argument('--lidar-blind', action='store_true',
                    help='cones too short for the lidar to see (camera-only fallback)')
    ap.add_argument('--out', default='hw9_output/sim')
    args = ap.parse_args(argv)

    cfg = MissionConfig(colors=list(args.colors), output_dir=args.out,
                        contact_range=CONTACT_RANGE,
                        camera_hfov_deg=args.true_hfov + args.hfov_error)
    bearings = [35.0, 160.0, 275.0]
    cones = [(c, args.radius * math.cos(math.radians(b)), args.radius * math.sin(math.radians(b)))
             for c, b in zip(args.colors, bearings)]
    robot = SimRobot(cfg, cones, start=(0.0, 0.0, math.radians(args.start_yaw)),
                     hfov_deg=args.true_hfov, gyro_scale=1.0 + args.gyro_error,
                     cam_hz=args.cam_hz, cam_latency=args.cam_latency,
                     lidar_blind=args.lidar_blind)
    print('Simulated cones: ' + ', '.join(f'{c} ({x:+.2f}, {y:+.2f})' for c, x, y in cones))
    print(f'contact_range for the simulated geometry: {CONTACT_RANGE:.3f} m')

    hunter = ConeHunter(robot, cfg)
    robot.sleep(1.0)
    hunter.task1()

    # Task 1 positions are in the odometry frame, which rotates away from the
    # true one by gyro_error * (angle turned). Check distance and bearing
    # separately and allow that drift in the bearing.
    failures = []
    print('\nTask 1 error (estimated vs true, from the start position):')
    yaw0 = math.radians(args.start_yaw)
    for colour, x, y in cones:
        c = hunter.cones.get(colour)
        if c is None or c.xy is None:
            failures.append(f'task 1: {colour} not located')
            print(f'  {colour}: not located')
            continue
        d_err = math.hypot(*c.xy) - math.hypot(x, y)
        true_rel = (math.atan2(y, x) - yaw0) % (2 * math.pi)
        b_err = math.degrees(math.atan2(math.sin(math.atan2(c.xy[1], c.xy[0]) - math.atan2(y, x)),
                                        math.cos(math.atan2(c.xy[1], c.xy[0]) - math.atan2(y, x))))
        b_tol = 2.0 + abs(args.gyro_error) * math.degrees(true_rel)
        print(f'  {colour}: distance {d_err * 100:+.1f} cm, bearing {b_err:+.1f} deg '
              f'(gyro drift allowance {b_tol:.1f} deg)')
        if abs(d_err) > 0.05 or abs(b_err) > b_tol:
            failures.append(f'task 1: {colour} off by {d_err:+.2f} m / {b_err:+.1f} deg')

    rows = []
    for colour in args.order:
        res = hunter.drive_to(colour)
        gap = robot.gap_to(colour)
        rows.append((colour, res, gap))
        if not res.ok:
            failures.append(f'task 2: {colour}: {res.message}')
        elif not 0.010 <= gap <= 0.040:
            failures.append(f'task 2: {colour}: true gap {gap * 100:.1f} cm')

    print('\nTask 2 summary (target gap 2.5 cm):')
    for colour, res, gap in rows:
        est = f'{res.gap * 100:5.1f}' if res.gap is not None else '    ?'
        print(f'  {colour:<7} {"ok " if res.ok else "FAIL"}  true gap {gap * 100:5.1f} cm,'
              f' estimated {est} cm')
    print(f'Closest the robot ever came to any cone: {robot.min_gap * 100:.1f} cm; '
          f'bumper presses: {robot.bump_count}; simulated time {robot.t:.0f} s')
    if robot.bump_count:
        failures.append('touched a cone')
    if failures:
        print('\nFAILED:\n  ' + '\n  '.join(failures))
        return 1
    print('\nPASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
