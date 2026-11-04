"""
HW9 mission: Task 1 (turn 360 degrees and locate three coloured cones) and
Task 2 (repeatedly drive to the cone the user names, stopping ~1 inch short).

The camera says *which* object is which colour; the lidar says exactly
*where* it is; odometry keeps track of the robot between the two.

The logic only talks to a `robot` object, so the same code drives the real
TurtleBot (ros_robot.RosRobot) and the offline simulator (sim_test.SimRobot):

    robot.now(), robot.sleep(dt)            clock (wall time or simulated)
    robot.pose()                            latest odometry (x, y, yaw); yaw unwrapped
    robot.latest_frame(), robot.pop_frames()  camera Frames with colour detections
    robot.latest_scan()                     lidar Scans
    robot.send_cmd(v, w), robot.stop()      velocity commands (m/s, rad/s)
    robot.bump_count                        grows whenever the bumper is pressed
"""

import math
import os
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from hw9pkg.geometry import (circular_mean, find_cluster, group_angles,
                             group_points, wrap)
from hw9pkg.vision import (annotate, draw_colour, pixel_offset,
                           pixel_to_bearing, put_label)


@dataclass
class MissionConfig:
    """Every field is also a ROS parameter of the same name."""
    colors: list = field(default_factory=lambda: ['orange', 'green', 'blue'])
    # camera
    camera_hfov_deg: float = 42.0     # TB4 250x250 preview = centre crop of the 69 deg image
    min_blob_fraction: float = 0.0008  # smallest blob, as a fraction of the image area
    # lidar
    laser_yaw_offset: float = math.pi / 2   # TB4 lidar is mounted +90 deg (hw6)
    laser_x_offset: float = -0.04     # lidar position ahead of base_link (m)
    min_valid_range: float = 0.16     # closer returns are the robot itself
    max_cone_range: float = 2.5       # cones further than this are ignored (m)
    cone_radius: float = 0.04         # cone radius at lidar height (position estimates only)
    # task 1
    scan_degrees: float = 360.0
    scan_speed: float = 0.30          # rad/s (slower if the camera frame rate is low)
    # task 2: stop when the lidar range to the cone is contact_range + target_gap
    contact_range: float = 0.28       # lidar range when the bumper just touches a cone. CALIBRATE
    target_gap: float = 0.025         # 1 inch
    max_speed: float = 0.15           # m/s
    min_speed: float = 0.025          # m/s, final creep
    slow_gain: float = 0.6            # speed = slow_gain * remaining distance
    turn_speed: float = 0.6           # rad/s
    search_speed: float = 0.4         # rad/s
    backoff_distance: float = 0.08    # reverse this far from a cone before turning away
    stop_latency: float = 0.15        # s of travel allowed for command/sensor delay
    output_dir: str = 'hw9_output'


class MissionError(Exception):
    """A problem the user has to fix (no data, robot not moving, ...)."""


@dataclass
class Scan:
    seq: int
    t: float                 # receive time (robot.now())
    bearings: np.ndarray     # robot orientation, from the lidar (rad)
    ranges: np.ndarray       # m
    pose: tuple              # (x, y, yaw) of the robot when the scan was taken
    laser_x_offset: float

    def origin(self):
        x, y, yaw = self.pose
        return x + self.laser_x_offset * math.cos(yaw), y + self.laser_x_offset * math.sin(yaw)

    def to_world(self, bearing, rng):
        ox, oy = self.origin()
        a = self.pose[2] + bearing
        return ox + rng * math.cos(a), oy + rng * math.sin(a)


@dataclass
class Frame:
    seq: int
    t: float                 # receive time (robot.now())
    bgr: np.ndarray
    detections: dict         # colour -> Blob or None
    pose: tuple              # (x, y, yaw) of the robot when the image was taken
    scan: Optional[Scan]     # latest scan when the image arrived

    @property
    def width(self):
        return self.bgr.shape[1]

    @property
    def height(self):
        return self.bgr.shape[0]


@dataclass
class Sighting:
    colour: str
    bearing: float           # odom-frame bearing of the blob from the robot (rad)
    offset: float            # blob position in the image, -1 .. +1
    area: int
    xy: Optional[tuple] = None    # cone centre (odom frame), if the lidar saw it
    rng: Optional[float] = None
    frame: Optional[Frame] = None
    blob: object = None


@dataclass
class Cone:
    colour: str
    xy: Optional[tuple]      # centre in the odom frame (None: camera only)
    bearing: float           # odom-frame bearing from the start position
    sightings: int
    frame: Optional[Frame] = None
    blob: object = None


@dataclass
class Result:
    ok: bool
    message: str
    lidar_range: Optional[float] = None
    gap: Optional[float] = None


CONTROL_DT = 0.05
TURN_GAIN = 1.5                     # rad/s per rad of heading error
MIN_TURN = 0.12                     # rad/s
CENTRE_TOL = math.radians(2.0)
ASSOC_MAX_OFFSET = 0.35             # only blobs this close to the image centre get a lidar range
ASSOC_WINDOW = math.radians(10.0)   # camera bearing -> lidar cluster search window
TRACK_WINDOW = math.radians(10.0)
TRACK_TOL = 0.10                    # m, predicted vs measured range while tracking
TRACK_TIMEOUT = 1.0                 # s without a lidar update -> track lost
CAMERA_ONLY_SPEED = 0.05
APPROACH_TIMEOUT = 90.0
SNAPSHOT_OFFSET = 0.15


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def fmt_bearing(rel):
    d = math.degrees(wrap(rel))
    if abs(d) < 1.0:
        return 'straight ahead'
    return f'{abs(d):.0f} deg {"left" if d > 0 else "right"}'


class ConeHunter:
    def __init__(self, robot, cfg: MissionConfig, say=None):
        self.robot = robot
        self.cfg = cfg
        self.hfov = math.radians(cfg.camera_hfov_deg)
        self.cones = {}
        self.start_pose = None
        self.start_scan = None
        self.parked_at = None
        self.last_result = None
        self._say = say or (lambda m: print(m, flush=True))
        os.makedirs(cfg.output_dir, exist_ok=True)

    def say(self, msg):
        self._say(msg)

    # ============================================================ Task 1
    def task1(self):
        r, cfg = self.robot, self.cfg
        self.say(f'\n=== Task 1: turning {cfg.scan_degrees:.0f} degrees to find the '
                 f'{", ".join(cfg.colors)} cones ===')
        self.start_pose = r.pose()
        r.pop_frames()
        sightings, spotted = [], set()

        # Look before turning (also measures the camera frame rate).
        t0 = r.now()
        n_frames = 0
        while r.now() - t0 < 1.0:
            r.sleep(CONTROL_DT)
            frames = r.pop_frames()
            n_frames += len(frames)
            self._observe(frames, sightings, spotted)
        self.start_scan = r.latest_scan()
        fps = n_frames / max(r.now() - t0, 1e-3)
        if fps < 0.5:
            raise MissionError('No camera images are arriving.')
        # At least one image per 2.5 degrees of turning.
        speed = clip(fps * math.radians(2.5), 0.08, cfg.scan_speed)
        self.say(f'Camera at {fps:.1f} fps -> turning at {math.degrees(speed):.0f} deg/s.')

        target = math.radians(cfg.scan_degrees)
        yaw_start = r.pose()[2]
        t0 = r.now()
        max_time = target / speed * 2.5 + 10.0
        while True:
            turned = r.pose()[2] - yaw_start
            if turned >= target:
                break
            elapsed = r.now() - t0
            if elapsed > 4.0 and turned < math.radians(5.0):
                r.stop()
                raise MissionError('The robot is not turning (odometry did not change). '
                                   'Is it undocked, and is /cmd_vel reaching it?')
            if elapsed > max_time:
                self.say('warning: the turn took too long; stopping it early.')
                break
            remaining = target - turned
            w = speed if remaining > 0.25 else max(0.08, speed * remaining / 0.25)
            r.send_cmd(0.0, w)
            r.sleep(CONTROL_DT)
            self._observe(r.pop_frames(), sightings, spotted)
        r.stop()
        t0 = r.now()
        while r.now() - t0 < 0.6:          # images still in flight
            r.sleep(CONTROL_DT)
            self._observe(r.pop_frames(), sightings, spotted)
        self.say(f'Turned {math.degrees(r.pose()[2] - yaw_start):.0f} degrees.')

        self.cones = self._analyse(sightings)
        self._report()
        return self.cones

    def _observe(self, frames, sightings, spotted):
        cfg = self.cfg
        for f in frames:
            for colour in cfg.colors:
                blob = f.detections.get(colour)
                if blob is None:
                    continue
                off = pixel_offset(blob.cx, f.width)
                bearing = f.pose[2] + pixel_to_bearing(blob.cx, f.width, self.hfov)
                s = Sighting(colour, bearing, off, blob.area)
                if abs(off) <= ASSOC_MAX_OFFSET and f.scan is not None:
                    sc = f.scan
                    cl = find_cluster(sc.bearings, sc.ranges, wrap(bearing - sc.pose[2]),
                                      ASSOC_WINDOW, max_range=cfg.max_cone_range)
                    if cl is not None and cl.compact:
                        s.rng = cl.range
                        s.xy = sc.to_world(cl.bearing, cl.range + cfg.cone_radius)
                if abs(off) <= SNAPSHOT_OFFSET:
                    s.frame, s.blob = f, blob
                sightings.append(s)
                if s.xy is not None and colour not in spotted:
                    spotted.add(colour)
                    x0, y0, yaw0 = self.start_pose
                    rel = math.atan2(s.xy[1] - y0, s.xy[0] - x0) - yaw0
                    dist = math.hypot(s.xy[0] - x0, s.xy[1] - y0)
                    self.say(f'  spotted the {colour.upper()} cone: {fmt_bearing(rel)}, '
                             f'{dist:.2f} m away')

    def _analyse(self, sightings):
        x0, y0, _ = self.start_pose
        cones = {}
        for colour in self.cfg.colors:
            mine = [s for s in sightings if s.colour == colour]
            located = [s for s in mine if s.xy is not None]
            if located:
                groups = group_points([s.xy for s in located], 0.3)
                best = max(groups, key=lambda g: (len(g), np.mean([located[i].area for i in g])))
                members = [located[i] for i in best]
                xy = (float(np.median([s.xy[0] for s in members])),
                      float(np.median([s.xy[1] for s in members])))
                bearing = math.atan2(xy[1] - y0, xy[0] - x0)
                near = [s for s in mine if abs(wrap(s.bearing - bearing)) < math.radians(12)]
            elif mine:
                groups = group_angles([s.bearing for s in mine], math.radians(10))
                near = [mine[i] for i in max(groups, key=len)]
                bearing = circular_mean([s.bearing for s in near])
                xy = None
            else:
                continue
            snaps = [s for s in near if s.frame is not None] or \
                    [s for s in mine if s.frame is not None]
            pick = min(snaps, key=lambda s: abs(s.offset)) if snaps else None
            cones[colour] = Cone(colour, xy, bearing, len(near),
                                 pick.frame if pick else None, pick.blob if pick else None)

        # Two colours on the same object means overlapping HSV ranges.
        names = [c for c in cones if cones[c].xy is not None]
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if a in cones and b in cones and \
                        math.dist(cones[a].xy, cones[b].xy) < 0.25:
                    weak = a if cones[a].sightings < cones[b].sightings else b
                    strong = b if weak == a else a
                    self.say(f'warning: {a} and {b} were detected on the same cone; keeping '
                             f'{strong}. Tighten the HSV ranges with cone_calib.')
                    del cones[weak]
        return cones

    def _start_frame_xy(self, xy):
        x0, y0, yaw0 = self.start_pose
        dx, dy = xy[0] - x0, xy[1] - y0
        return (math.cos(yaw0) * dx + math.sin(yaw0) * dy,
                -math.sin(yaw0) * dx + math.cos(yaw0) * dy)

    def _report(self):
        cfg = self.cfg
        x0, y0, yaw0 = self.start_pose
        self.say(f'\n=== Task 1 result: found {len(self.cones)} of {len(cfg.colors)} cones ===')
        self.say('  colour    direction           distance   position (x ahead, y left)')
        for colour in cfg.colors:
            c = self.cones.get(colour)
            if c is None:
                self.say(f'  {colour:<9} NOT FOUND')
                continue
            rel = wrap(c.bearing - yaw0)
            if c.xy is not None:
                fx, fy = self._start_frame_xy(c.xy)
                self.say(f'  {colour:<9} {fmt_bearing(rel):<19} {math.hypot(fx, fy):5.2f} m'
                         f'    ({fx:+.2f}, {fy:+.2f}) m   [{c.sightings} sightings]')
            else:
                self.say(f'  {colour:<9} {fmt_bearing(rel):<19}   ?      camera only, the '
                         f'lidar did not see it   [{c.sightings} sightings]')
        located = [c for c in cfg.colors if c in self.cones and self.cones[c].xy is not None]
        if len(located) >= 2:
            pairs = [f'{a}-{b} {math.dist(self.cones[a].xy, self.cones[b].xy):.2f} m'
                     for i, a in enumerate(located) for b in located[i + 1:]]
            self.say('  cone spacing: ' + ', '.join(pairs))
        self._save_task1_images()

    def _save_task1_images(self):
        out = self.cfg.output_dir
        saved = []
        for colour, c in self.cones.items():
            if c.frame is None:
                continue
            fx_txt = ''
            if c.xy is not None:
                fx, fy = self._start_frame_xy(c.xy)
                fx_txt = f' {math.hypot(fx, fy):.2f} m'
            img = annotate(c.frame.bgr, c.frame.detections, highlight=colour,
                           text=f'{colour}: {fmt_bearing(c.bearing - self.start_pose[2])}{fx_txt}')
            path = os.path.join(out, f'task1_{colour}.png')
            cv2.imwrite(path, img)
            saved.append(path)
        path = os.path.join(out, 'task1_map.png')
        cv2.imwrite(path, self._draw_map())
        saved.append(path)
        self.say('  saved ' + ', '.join(saved))

    def _draw_map(self, size=500, scale=80.0):
        """Top-down view in the start frame: x ahead = up, y left = left."""
        img = np.full((size, size, 3), 255, np.uint8)
        c0 = size // 2

        def px(xy):
            fx, fy = self._start_frame_xy(xy)
            return int(round(c0 - fy * scale)), int(round(c0 - fx * scale))

        for k in range(-5, 6):                       # 0.5 m grid
            p = int(c0 + k * 0.5 * scale)
            cv2.line(img, (p, 0), (p, size), (235, 235, 235), 1)
            cv2.line(img, (0, p), (size, p), (235, 235, 235), 1)
        s = self.start_scan
        if s is not None:
            for b, rr in zip(s.bearings, s.ranges):
                cv2.circle(img, px(s.to_world(b, rr)), 1, (120, 120, 120), -1)
        x0, y0, yaw0 = self.start_pose
        cv2.circle(img, px((x0, y0)), int(0.17 * scale), (60, 60, 60), 2)
        cv2.arrowedLine(img, px((x0, y0)),
                        px((x0 + 0.3 * math.cos(yaw0), y0 + 0.3 * math.sin(yaw0))),
                        (60, 60, 60), 2, tipLength=0.3)
        for colour, c in self.cones.items():
            col = draw_colour(colour)
            if c.xy is not None:
                cv2.circle(img, px(c.xy), int(0.07 * scale), col, -1)
                cv2.circle(img, px(c.xy), int(0.07 * scale), (0, 0, 0), 1)
                fx, fy = self._start_frame_xy(c.xy)
                u, v = px(c.xy)
                put_label(img, f'{colour} {math.hypot(fx, fy):.2f} m', (u + 10, v - 10), col, 0.5, outline=False)
            else:
                end = (x0 + 2.0 * math.cos(c.bearing), y0 + 2.0 * math.sin(c.bearing))
                cv2.line(img, px((x0, y0)), px(end), col, 1)
                u, v = px(end)
                put_label(img, f'{colour}?', (u, v), col, 0.5, outline=False)
        put_label(img, 'Task 1: cones found (grey = lidar, grid 0.5 m)', (8, 20), (60, 60, 60), 0.5, outline=False)
        return img

    # ============================================================ Task 2
    def task2(self, ask=input):
        colours = self.cfg.colors
        self.say('\n=== Task 2: drive to a cone ===')
        while True:
            try:
                ans = ask(f'\nWhich cone should I drive to? ({", ".join(colours)}; q = quit): ')
            except EOFError:
                break
            choice = self.match_colour(ans)
            if choice == 'quit':
                break
            if choice is None:
                self.say(f'  {ans.strip()!r} is not one of: {", ".join(colours)}')
                continue
            self.drive_to(choice)
        self.say('Done.')

    def match_colour(self, text):
        colours = self.cfg.colors
        t = text.strip().lower()
        if t in ('q', 'quit', 'exit'):
            return 'quit'
        if not t:
            return None
        if t.isdigit() and 1 <= int(t) <= len(colours):
            return colours[int(t) - 1]
        if t in colours:
            return t
        pref = [c for c in colours if c.startswith(t)]
        return pref[0] if len(pref) == 1 else None

    def drive_to(self, colour):
        r, cfg = self.robot, self.cfg
        if self.parked_at == colour and self.last_result is not None:
            self.say(f'Already parked at the {colour} cone.')
            return self.last_result
        self.say(f'\n--> Driving to the {colour.upper()} cone')
        bumps0 = r.bump_count
        if self.parked_at is not None and cfg.backoff_distance > 0:
            self.say(f'  backing away {cfg.backoff_distance * 100:.0f} cm from the '
                     f'{self.parked_at} cone')
            self._drive_straight(-cfg.backoff_distance, 0.08)
        self.parked_at = None

        cone = self.cones.get(colour)
        if cone is not None:
            x, y, _ = r.pose()
            yaw = math.atan2(cone.xy[1] - y, cone.xy[0] - x) if cone.xy else cone.bearing
            self.say(f'  turning toward where the {colour} cone was seen')
            self._turn_to(yaw)
        if not self._acquire(colour):
            return self._finish(colour, Result(False, 'could not find it with the camera'))
        if not self._centre(colour):
            return self._finish(colour, Result(False, 'could not centre it in the camera image'))
        return self._finish(colour, self._approach(colour, cone, bumps0))

    def _finish(self, colour, res):
        if res.ok:
            self.parked_at = colour
            self.say(f'  ARRIVED at the {colour.upper()} cone: lidar range '
                     f'{res.lidar_range:.3f} m -> estimated gap {res.gap * 100:.1f} cm '
                     f'({res.gap / 0.0254:.1f} in)')
        else:
            self.say(f'  FAILED to reach the {colour} cone: {res.message}')
        self.last_result = res
        return res

    # ------------------------------------------------------------ motion
    def _turn_to(self, target_yaw, tol=math.radians(2.0), timeout=20.0):
        r = self.robot
        t0 = r.now()
        while r.now() - t0 < timeout:
            err = wrap(target_yaw - r.pose()[2])
            if abs(err) <= tol:
                break
            r.send_cmd(0.0, math.copysign(clip(TURN_GAIN * abs(err), MIN_TURN,
                                               self.cfg.turn_speed), err))
            r.sleep(CONTROL_DT)
        r.stop()

    def _drive_straight(self, distance, speed):
        r = self.robot
        x0, y0, _ = r.pose()
        t0 = r.now()
        limit = abs(distance) / speed * 2.0 + 3.0
        while True:
            x, y, _ = r.pose()
            if math.hypot(x - x0, y - y0) >= abs(distance) - 0.005:
                break
            if r.now() - t0 > limit:
                self.say('  warning: could not finish the straight move')
                break
            r.send_cmd(math.copysign(speed, distance), 0.0)
            r.sleep(CONTROL_DT)
        r.stop()

    def _wait_frame(self, after, timeout=2.0):
        r = self.robot
        t_end = r.now() + timeout
        while r.now() < t_end:
            f = r.latest_frame()
            if f is not None and f.t > after:
                return f
            r.sleep(0.03)
        return None

    def _wait_scan(self, after, timeout=2.0):
        r = self.robot
        t_end = r.now() + timeout
        while r.now() < t_end:
            s = r.latest_scan()
            if s is not None and s.t > after:
                return s
            r.sleep(0.03)
        return None

    def _blob_target(self, f, colour):
        """Odom-frame bearing of the colour blob in frame f, or None."""
        blob = f.detections.get(colour) if f is not None else None
        if blob is None:
            return None
        return f.pose[2] + pixel_to_bearing(blob.cx, f.width, self.hfov)

    def _acquire(self, colour):
        """Make sure the cone is in the camera view, spinning to search if needed."""
        r, cfg = self.robot, self.cfg
        f = self._wait_frame(after=r.now() + 0.15)
        if f is None:
            raise MissionError('No camera images are arriving.')
        if f.detections.get(colour) is not None:
            return True
        self.say(f'  the {colour} cone is not in view; searching...')
        yaw_start = r.pose()[2]
        t0 = r.now()
        r.pop_frames()
        while r.pose()[2] - yaw_start < 2 * math.pi + 0.3:
            if r.now() - t0 > 2 * math.pi / cfg.search_speed * 2.5 + 5.0:
                break
            r.send_cmd(0.0, cfg.search_speed)
            r.sleep(CONTROL_DT)
            if any(fr.detections.get(colour) is not None for fr in r.pop_frames()):
                r.stop()
                return True
        r.stop()
        return False

    def _centre(self, colour, timeout=15.0):
        """Turn until the cone is in the middle of the camera image."""
        r = self.robot
        t0 = r.now()
        target, last_seen = None, None
        while r.now() - t0 < timeout:
            f = r.latest_frame()
            tgt = self._blob_target(f, colour)
            if tgt is not None:
                target, last_seen = tgt, f.t
            if target is None or r.now() - last_seen > 2.0:
                r.stop()
                return False
            err = wrap(target - r.pose()[2])
            if abs(err) > CENTRE_TOL:
                r.send_cmd(0.0, math.copysign(clip(TURN_GAIN * abs(err), MIN_TURN,
                                                   self.cfg.turn_speed), err))
                r.sleep(CONTROL_DT)
                continue
            # Looks centred: confirm with an image taken after stopping.
            r.stop()
            f2 = self._wait_frame(after=r.now() + 0.25)
            tgt = self._blob_target(f2, colour)
            if tgt is None:
                continue
            target, last_seen = tgt, f2.t
            if abs(wrap(target - r.pose()[2])) <= 1.5 * CENTRE_TOL:
                return True
        r.stop()
        return False

    def _polar(self, pose, point):
        """(bearing, range) of a world point from the lidar at `pose`."""
        x, y, yaw = pose
        ox = x + self.cfg.laser_x_offset * math.cos(yaw)
        oy = y + self.cfg.laser_x_offset * math.sin(yaw)
        return (wrap(math.atan2(point[1] - oy, point[0] - ox) - yaw),
                math.hypot(point[0] - ox, point[1] - oy))

    def _approach(self, colour, cone, bumps0):
        """
        Drive at the cone until the lidar range to it is contact_range + target_gap.

        The cone's nearest point at lidar height is tracked as a fixed point in
        the odom frame. Each scan re-measures it, and odometry moves it between
        scans, so the remaining distance is always up to date. The camera only
        picks the right cluster and checks that the lidar stays on it.
        """
        r, cfg = self.robot, self.cfg
        stop_range = cfg.contact_range + cfg.target_gap
        t0 = r.now()
        track, track_t, last_range = None, None, None
        last_scan_seq = -1
        v_cmd = 0.0
        warned = False
        lost_since = None
        tr = None

        while True:
            now = r.now()
            if now - t0 > APPROACH_TIMEOUT:
                r.stop()
                return Result(False, 'timed out')
            if r.bump_count != bumps0:
                r.stop()
                return Result(False, 'the bumper was pressed: it touched something')
            pose = r.pose()

            f = r.latest_frame()
            cam_b, blob = None, None
            if f is not None and now - f.t < 0.6:
                tgt = self._blob_target(f, colour)
                if tgt is not None:
                    cam_b, blob = wrap(tgt - pose[2]), f.detections[colour]

            s = r.latest_scan()
            if s is not None and s.seq != last_scan_seq:
                last_scan_seq = s.seq
                cl = None
                if track is not None:
                    pb, pr = self._polar(s.pose, track)
                    cl = find_cluster(s.bearings, s.ranges, pb, TRACK_WINDOW,
                                      expected=pr, tol=TRACK_TOL)
                elif cam_b is not None:
                    exp = None
                    if cone is not None and cone.xy is not None:
                        exp = self._polar(s.pose, cone.xy)[1] - cfg.cone_radius
                    cl = find_cluster(s.bearings, s.ranges, wrap(cam_b + pose[2] - s.pose[2]),
                                      ASSOC_WINDOW, expected=exp,
                                      tol=None if exp is None else 0.5,
                                      max_range=cfg.max_cone_range)
                    if cl is not None and not cl.compact:
                        cl = None
                if cl is not None:
                    track, track_t = s.to_world(cl.bearing, cl.range), now

            if track is not None:
                tb, tr = self._polar(pose, track)
                if now - track_t > TRACK_TIMEOUT:
                    track = None
                elif cam_b is not None and tr > 0.6 and abs(wrap(cam_b - tb)) > math.radians(15):
                    track = None          # lidar is on something else; re-associate

            if track is not None:
                last_range = tr
                remaining = tr - stop_range
                if remaining <= v_cmd * cfg.stop_latency:
                    break
                v = clip(cfg.slow_gain * remaining, cfg.min_speed, cfg.max_speed)
                if abs(tb) > math.radians(20):
                    v = 0.0
                elif abs(tb) > math.radians(8):
                    v *= 0.5
                w = clip(TURN_GAIN * tb, -cfg.turn_speed, cfg.turn_speed)
                lost_since = None
            elif cam_b is not None:
                if not warned:
                    self.say('  the lidar does not see the cone yet; steering by camera')
                    warned = True
                if blob.bottom >= f.height - 2 and blob.h >= 0.5 * f.height:
                    r.stop()
                    return Result(False, 'the lidar never saw the cone; stopped when it '
                                  'filled the camera view (gap unknown)')
                near = last_range is not None and last_range < stop_range + 0.2
                v = CAMERA_ONLY_SPEED if abs(cam_b) < math.radians(10) and not near else 0.0
                w = clip(TURN_GAIN * cam_b, -cfg.turn_speed, cfg.turn_speed)
                lost_since = None
            else:
                v = w = 0.0
                if lost_since is None:
                    lost_since = now
                if now - lost_since > 3.0:
                    r.stop()
                    return Result(False, 'lost sight of the cone')
            r.send_cmd(v, w)
            v_cmd = v
            r.sleep(CONTROL_DT)

        r.stop()
        # Measure the final distance from a scan taken after stopping.
        final = tr
        s = self._wait_scan(after=r.now() + 0.3)
        if s is not None:
            pb, pr = self._polar(s.pose, track)
            cl = find_cluster(s.bearings, s.ranges, pb, TRACK_WINDOW, expected=pr, tol=TRACK_TOL)
            if cl is not None:
                final = cl.range
                track = s.to_world(cl.bearing, cl.range)
        # Remember the refined cone position for the next request.
        ox, oy = self._lidar_xy(r.pose())
        d = math.hypot(track[0] - ox, track[1] - oy)
        k = (d + cfg.cone_radius) / d
        xy = (ox + k * (track[0] - ox), oy + k * (track[1] - oy))
        old = self.cones.get(colour)
        self.cones[colour] = Cone(colour, xy, math.atan2(xy[1] - oy, xy[0] - ox),
                                  old.sightings if old else 0,
                                  old.frame if old else None, old.blob if old else None)
        return Result(True, 'arrived', final, final - cfg.contact_range)

    def _lidar_xy(self, pose):
        x, y, yaw = pose
        return (x + self.cfg.laser_x_offset * math.cos(yaw),
                y + self.cfg.laser_x_offset * math.sin(yaw))
