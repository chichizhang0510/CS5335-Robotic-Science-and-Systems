"""
HW9: find three coloured traffic cones, then drive to the one the user names.

  ros2 run hw9pkg cone_finder --ros-args -p colors:=orange,green,blue -p touch_range:=0.205

Task 1  Turn a full circle in steps of sweep_step_deg. At each stop take a
        fresh camera frame and lidar scan. Each colour's largest blob gives a
        rough direction (pixel column -> angle); the nearest small lidar
        object in that direction gives the exact direction and distance.
        The sighting nearest the image centre is kept for each colour and
        its position stored in the odom frame. Prints the colour, bearing
        and distance of every cone.
Task 2  Loop: ask for a colour, back away from the last cone, turn toward the
        stored position, find the cone again with the camera, and close in
        by measure -> drive steps: first to final_stage (0.25 m) short, then
        the rest, re-measuring with the lidar while stopped, until the lidar
        says the bumper is stop_gap (2.5 cm) from the cone.

touch_range (lidar range when the cone touches the bumper) must be measured
on your robot with `ros2 run hw9pkg calibrate lidar`; see the README.
Annotated camera frames from the sweep are saved in output_dir.
"""

import math
import os
import statistics
from dataclasses import dataclass

import cv2
import rclpy

from hw9pkg.color_detect import annotate, detect, load_hsv
from hw9pkg.lidar_objects import LidarObject, find_object, wrap_angle
from hw9pkg.robot import MotionError, RobotIO, shutdown, spin_in_background


@dataclass
class Sighting:
    x: float          # robot pose when the picture was taken (odom frame)
    y: float
    yaw: float
    camera: float     # rad, robot frame, from the pixel column
    obj: LidarObject  # lidar object matched in that direction


@dataclass
class Cone:
    color: str
    x: float          # odom frame, nearest face of the cone
    y: float
    bearing: float    # rad from the starting heading, when it was found
    range: float      # m from the lidar, when it was found


class ConeFinder(RobotIO):
    def __init__(self):
        super().__init__('cone_finder')

        self.declare_parameter('colors', 'orange,green,blue')
        self.declare_parameter('hsv_file', 'hsv_ranges.yaml')
        self.declare_parameter('min_area_frac', 0.001)        # min blob area / image area
        self.declare_parameter('sweep_step_deg', 24.0)        # < camera FOV, so views overlap
        self.declare_parameter('max_cone_range', 3.0)         # m, ignore anything further
        self.declare_parameter('max_cone_width', 0.45)        # m, wider lidar objects are walls
        self.declare_parameter('lidar_window_deg', 12.0)      # search +- this around the camera angle
        self.declare_parameter('close_window_deg', 20.0)      # same, lidar-only tracking up close
        self.declare_parameter('stop_gap', 0.025)             # m, bumper-to-cone gap to stop at
        self.declare_parameter('arrive_tolerance', 0.008)     # m
        self.declare_parameter('align_tolerance_deg', 2.0)
        self.declare_parameter('final_stage', 0.25)           # m, stop this far short and re-measure
        self.declare_parameter('camera_handoff_range', 0.55)  # m, closer than this: lidar only
        self.declare_parameter('backoff_distance', 0.30)      # m, back away from a reached cone
        self.declare_parameter('scans_per_measure', 3)
        self.declare_parameter('max_steps', 12)
        self.declare_parameter('output_dir', './hw9_output')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.colors = [c.strip().lower() for c in str(gp('colors')).split(',') if c.strip()]
        self.hsv, source = load_hsv(gp('hsv_file'))
        unknown = [c for c in self.colors if c not in self.hsv]
        if unknown:
            raise ValueError(f'no HSV range for {unknown}; known colours: {sorted(self.hsv)}')
        self.min_area_frac = float(gp('min_area_frac'))
        self.sweep_step = math.radians(float(gp('sweep_step_deg')))
        self.max_range = float(gp('max_cone_range'))
        self.max_width = float(gp('max_cone_width'))
        self.window = math.radians(float(gp('lidar_window_deg')))
        self.close_window = math.radians(float(gp('close_window_deg')))
        self.stop_gap = float(gp('stop_gap'))
        self.arrive_tol = float(gp('arrive_tolerance'))
        self.align_tol = math.radians(float(gp('align_tolerance_deg')))
        self.final_stage = float(gp('final_stage'))
        self.handoff = float(gp('camera_handoff_range'))
        self.backoff = float(gp('backoff_distance'))
        self.scans_per_measure = int(gp('scans_per_measure'))
        self.max_steps = int(gp('max_steps'))
        self.out_dir = os.path.expanduser(gp('output_dir'))
        os.makedirs(self.out_dir, exist_ok=True)

        self.cones = {}
        self.start_yaw = None
        self.at_cone = False      # True right after reaching a cone

        self.get_logger().info(f'Colours: {self.colors}; HSV ranges from {source}')
        self.get_logger().info(f'touch_range = {self.touch_range:.3f} m, '
                               f'stopping at {self.touch_range + self.stop_gap:.3f} m')
        if len(self.colors) != 3:
            self.get_logger().warn(f'The assignment uses 3 cones; {len(self.colors)} colours set.')

    # ------------------------------------------------------------------ #
    # Task 1: turn 360 deg and locate each cone
    # ------------------------------------------------------------------ #
    def sweep(self, colors):
        n = max(1, math.ceil(2 * math.pi / self.sweep_step - 1e-6))
        step = 2 * math.pi / n
        _, _, yaw0 = self.pose()
        if self.start_yaw is None:
            self.start_yaw = yaw0
        print(f'\nTurning 360 deg in {n} steps of {math.degrees(step):.0f} deg, '
              f'looking for: {", ".join(colors)}')

        best, camera_only = {}, {}
        for k in range(n):
            frame = self.frame()
            pts, astep = self.points()
            x, y, yaw = self.pose()
            heading = math.degrees(wrap_angle(yaw - self.start_yaw))
            blobs = detect(frame, colors, self.hsv, self.min_area_frac)
            notes = []
            for color, blob in blobs.items():
                cam = self.camera_bearing(blob.u, frame.shape[1])
                obj = find_object(pts, cam, self.window, self.max_range, self.max_width, astep)
                if obj is None:
                    notes.append(f'{color} (camera {math.degrees(cam):+.0f} deg, no lidar object)')
                    camera_only.setdefault(color, wrap_angle(yaw + cam - self.start_yaw))
                    continue
                notes.append(f'{color} {obj.range:.2f} m at {math.degrees(obj.bearing):+.0f} deg')
                if color not in best or abs(cam) < abs(best[color].camera):
                    best[color] = Sighting(x, y, yaw, cam, obj)
            print(f'  look {k + 1:2d}/{n}  heading {heading:+6.1f} deg: '
                  + ('; '.join(notes) or 'no cone colours'))
            cv2.imwrite(os.path.join(self.out_dir, f'sweep_{k + 1:02d}.png'),
                        annotate(frame, blobs, f'heading {heading:+.0f}'))
            self.rotate_by(step)

        for color, s in best.items():
            b = s.yaw + s.obj.bearing
            self.cones[color] = Cone(color, s.x + s.obj.range * math.cos(b),
                                     s.y + s.obj.range * math.sin(b),
                                     wrap_angle(b - self.start_yaw), s.obj.range)
        for color in colors:
            if color not in best and color in camera_only:
                print(f'  ! {color} was seen by the camera at '
                      f'{math.degrees(camera_only[color]):+.0f} deg but the lidar found no '
                      f'object there (cone below the lidar, or further than {self.max_range} m?)')

    def report(self):
        lines = ['Cones found (bearing from the starting heading, + = left / counter-clockwise):']
        for color in self.colors:
            cone = self.cones.get(color)
            if cone is None:
                lines.append(f'  {color.upper():8s} NOT FOUND')
            else:
                lines.append(f'  {color.upper():8s} bearing {math.degrees(cone.bearing):+7.1f} deg'
                             f'   distance {cone.range:.2f} m')
        text = '\n'.join(lines)
        print('\n' + text)
        with open(os.path.join(self.out_dir, 'cones.txt'), 'w') as f:
            f.write(text + '\n')

    # ------------------------------------------------------------------ #
    # Task 2: drive to a named cone
    # ------------------------------------------------------------------ #
    def measure(self, color, use_camera):
        """(bearing rad, range m) of the colour's cone from the robot now, or None.

        With the camera: lidar object nearest to where the colour is seen.
        Without (up close, the cone fills the view): nearest small object
        straight ahead. Median over a few scans taken while stopped.
        """
        guess, window = 0.0, self.close_window
        if use_camera:
            frame = self.frame()
            blob = detect(frame, [color], self.hsv, self.min_area_frac).get(color)
            if blob is None:
                return None
            guess, window = self.camera_bearing(blob.u, frame.shape[1]), self.window
        bearings, ranges = [], []
        for _ in range(self.scans_per_measure):
            pts, astep = self.points()
            obj = find_object(pts, guess, window, self.max_range, self.max_width, astep)
            if obj is not None:
                bearings.append(obj.bearing)
                ranges.append(obj.range)
        if not ranges:
            return None
        return statistics.median(bearings), statistics.median(ranges)

    def look_around(self, color):
        """The cone is not in view: look 30 deg left, then 30 deg right."""
        for turn in (math.radians(30), math.radians(-60)):
            self.rotate_by(turn)
            m = self.measure(color, use_camera=True)
            if m is not None:
                return m
        self.rotate_by(math.radians(30))
        return None

    def go_to(self, color):
        if self.at_cone:
            print(f'Backing up {self.backoff:.2f} m from the last cone.')
            self.drive_by(-self.backoff, check_front=False)
            self.at_cone = False
        if color not in self.cones:
            print(f'The {color} cone was not located yet; turning around to look for it.')
            self.sweep([color])
            if color not in self.cones:
                print(f'Could not find the {color} cone.')
                return False

        cone = self.cones[color]
        x, y, yaw = self.pose()
        print(f'Turning toward the {color} cone.')
        self.rotate_by(wrap_angle(math.atan2(cone.y - y, cone.x - x) - yaw))
        m = self.measure(color, use_camera=True)
        if m is None:
            m = self.look_around(color)
        if m is None:
            print(f'Cannot see the {color} cone from here.')
            return False

        stop_range = self.touch_range + self.stop_gap
        for _ in range(self.max_steps):
            bearing, rng = m
            remaining = rng - stop_range
            if abs(remaining) <= self.arrive_tol:
                break
            if abs(bearing) > self.align_tol:
                self.rotate_by(bearing)
            else:
                # far away: stop final_stage short and re-measure; close: drive the rest
                move = remaining - self.final_stage if remaining > self.final_stage + 0.05 else remaining
                print(f'  {color}: {rng:.3f} m from the lidar, driving {move:+.3f} m')
                rng -= self.drive_by(move)
            m = self.measure(color, use_camera=rng > self.handoff)
            if m is None:     # camera lost it: trust the lidar straight ahead
                m = self.measure(color, use_camera=False)
            if m is None:
                print(f'Lost the {color} cone.')
                return False
        else:
            print(f'  Gave up refining after {self.max_steps} steps.')

        bearing, rng = m
        x, y, yaw = self.pose()
        b = yaw + bearing
        self.cones[color] = Cone(color, x + rng * math.cos(b), y + rng * math.sin(b),
                                 cone.bearing, cone.range)
        self.at_cone = True
        print(f'Arrived at the {color.upper()} cone: bumper gap about '
              f'{(rng - self.touch_range) * 100:.1f} cm (target {self.stop_gap * 100:.1f} cm).')
        return True

    def ask_loop(self):
        choices = '/'.join(self.colors)
        while True:
            answer = input(f'\nWhich cone? ({choices}, list, quit): ').strip().lower()
            if answer in ('q', 'quit', 'exit'):
                return
            if answer in ('l', 'list'):
                self.report()
                continue
            if answer not in self.colors:
                print(f'Type one of: {choices}')
                continue
            try:
                self.go_to(answer)
            except (MotionError, TimeoutError) as e:
                self.stop()
                self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = ConeFinder()
    executor = spin_in_background(node)
    try:
        node.wait_ready(camera=True)
        node.sweep(node.colors)     # Task 1
        node.report()
        node.ask_loop()             # Task 2
    except (KeyboardInterrupt, EOFError):
        print('\nStopped.')
    except (MotionError, TimeoutError, RuntimeError) as e:
        node.get_logger().error(str(e))
    finally:
        shutdown(node, executor)


if __name__ == '__main__':
    main()
