"""
ROS 2 side of HW9: the real TurtleBot behind the mission's robot interface.

Callbacks run in a background executor thread and only store the latest
data; the mission runs in the main thread and reads it. Every camera image
and lidar scan gets the odometry pose at its own time stamp (interpolated),
so the robot's turning between capture and arrival does not matter.
"""

import bisect
import threading
import time
from collections import deque
from dataclasses import fields

import cv2
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan

from hw9pkg.geometry import scan_to_polar, wrap, yaw_from_quaternion
from hw9pkg.mission import Frame, MissionConfig, MissionError, Scan
from hw9pkg.vision import (DEFAULT_HSV, compressed_msg_to_bgr, detect_all,
                           image_msg_to_bgr, min_blob_area, parse_boxes)

try:
    from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
except ImportError:                      # bumper monitoring is optional
    HazardDetection = HazardDetectionVector = None


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RosRobot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.cfg = self._declare_config()
        self.image_topic = self._param('image_topic', '/oakd/rgb/preview/image_raw')
        self.scan_topic = self._param('scan_topic', '/scan')
        self.odom_topic = self._param('odom_topic', '/odom')
        self.cmd_topic = self._param('cmd_vel_topic', '/cmd_vel')
        self.cmd_stamped = bool(self._param('cmd_vel_stamped', True))
        self.hazard_topic = self._param('hazard_topic', '/hazard_detection')
        self.colour_table = self._declare_colours()

        self._lock = threading.Lock()
        self._odom = deque(maxlen=1200)     # (stamp, x, y, unwrapped yaw)
        self._pose = None
        self._raw_yaw = None
        self._yaw_u = 0.0
        self._frame = None
        self._frames = deque(maxlen=200)
        self._frame_seq = 0
        self._scan = None
        self._scan_seq = 0
        self._warned = set()
        self.bump_count = 0

        self._compressed = 'compressed' in self.image_topic
        img_type = CompressedImage if self._compressed else Image
        self.create_subscription(img_type, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        if HazardDetectionVector is not None:
            self.create_subscription(HazardDetectionVector, self.hazard_topic,
                                     self._hazard_cb, qos_profile_sensor_data)
        else:
            self.get_logger().warn('irobot_create_msgs not found: bumper is not monitored')
        self._cmd_pub = self.create_publisher(
            TwistStamped if self.cmd_stamped else Twist, self.cmd_topic, 10)

    # ------------------------------------------------------ parameters
    def _param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return self.get_parameter(name).value

    def _declare_config(self):
        base = MissionConfig()
        values = {}
        for f in fields(MissionConfig):
            default = getattr(base, f.name)
            v = self._param(f.name, default)
            if isinstance(default, bool):
                v = bool(v)
            elif isinstance(default, float):
                v = float(v)
            elif isinstance(default, list):
                v = [str(x).lower() for x in v]
            values[f.name] = v
        cfg = MissionConfig(**values)
        if len(set(cfg.colors)) != len(cfg.colors) or not cfg.colors:
            raise MissionError(f'colors must be distinct names, got {cfg.colors}')
        return cfg

    def _declare_colours(self):
        table = {}
        for c in self.cfg.colors:
            name = f'hsv.{c}'
            if c in DEFAULT_HSV:
                self.declare_parameter(name, DEFAULT_HSV[c])
                val = self.get_parameter(name).value
            else:
                self.declare_parameter(name, Parameter.Type.INTEGER_ARRAY)
                try:
                    val = self.get_parameter(name).value
                except Exception:
                    val = None
                if not val:
                    raise MissionError(
                        f"No HSV range for colour '{c}'. Add 'hsv.{c}: [h_lo, h_hi, s_lo, "
                        f"s_hi, v_lo, v_hi]' to the params file (measure it with cone_calib).")
            table[c] = parse_boxes(val, c)
        return table

    # ------------------------------------------------------- callbacks
    def _warn_once(self, key, msg):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(msg)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        raw = yaw_from_quaternion(msg.pose.pose.orientation)
        st = stamp_to_sec(msg.header.stamp)
        with self._lock:
            if self._raw_yaw is None:
                self._yaw_u = raw
            else:
                self._yaw_u += wrap(raw - self._raw_yaw)
            self._raw_yaw = raw
            self._pose = (p.x, p.y, self._yaw_u)
            if not self._odom or st > self._odom[-1][0]:
                self._odom.append((st, p.x, p.y, self._yaw_u))

    def _pose_at(self, stamp):
        """Odometry pose at a message time stamp (linear interpolation)."""
        with self._lock:
            hist = list(self._odom)
            latest = self._pose
        if latest is None:
            return (0.0, 0.0, 0.0)            # before odometry: placeholder, discarded later
        if not hist or stamp <= 0.0:
            return latest
        if stamp < hist[0][0] - 0.5 or stamp > hist[-1][0] + 0.5:
            self._warn_once('clock', 'Camera/lidar time stamps do not match /odom time stamps; '
                                     'using the latest odometry for each message instead.')
            return latest
        times = [h[0] for h in hist]
        i = bisect.bisect_left(times, stamp)
        if i <= 0:
            return hist[0][1:]
        if i >= len(hist):
            return hist[-1][1:]
        a, b = hist[i - 1], hist[i]
        s = (stamp - a[0]) / (b[0] - a[0])
        return tuple(a[k] + s * (b[k] - a[k]) for k in (1, 2, 3))

    def _image_cb(self, msg):
        t = time.monotonic()
        try:
            bgr = compressed_msg_to_bgr(msg) if self._compressed else image_msg_to_bgr(msg)
        except Exception as e:                   # noqa: BLE001 - report and carry on
            self._warn_once('image', f'Cannot decode camera image: {e}')
            return
        if bgr is None:
            return
        if bgr.shape[1] > 480:                   # keep detection fast on large images
            k = 320.0 / bgr.shape[1]
            bgr = cv2.resize(bgr, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        h, w = bgr.shape[:2]
        dets = detect_all(bgr, self.colour_table, min_blob_area(w, h, self.cfg.min_blob_fraction))
        pose = self._pose_at(stamp_to_sec(msg.header.stamp))
        with self._lock:
            self._frame_seq += 1
            f = Frame(self._frame_seq, t, bgr, dets, pose, self._scan)
            self._frame = f
            self._frames.append(f)

    def _scan_cb(self, msg):
        t = time.monotonic()
        b, r = scan_to_polar(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                             msg.range_max, self.cfg.laser_yaw_offset, self.cfg.min_valid_range)
        pose = self._pose_at(stamp_to_sec(msg.header.stamp))
        with self._lock:
            self._scan_seq += 1
            self._scan = Scan(self._scan_seq, t, b, r, pose, self.cfg.laser_x_offset)

    def _hazard_cb(self, msg):
        for d in msg.detections:
            if d.type == HazardDetection.BUMP:
                self.bump_count += 1
                return
            if d.type == HazardDetection.BACKUP_LIMIT:
                self._warn_once('backup', 'Create 3 backup limit: it refuses to reverse further. '
                                          'Use -p backoff_distance:=0.0 if this repeats.')

    # ------------------------------------------- mission robot interface
    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        time.sleep(dt)

    def pose(self):
        with self._lock:
            return self._pose

    def latest_frame(self):
        with self._lock:
            return self._frame

    def latest_scan(self):
        with self._lock:
            return self._scan

    def pop_frames(self):
        with self._lock:
            out = list(self._frames)
            self._frames.clear()
        return out

    def send_cmd(self, v, w):
        if self.cmd_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            tw = msg.twist
        else:
            msg = Twist()
            tw = msg
        tw.linear.x = float(v)
        tw.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def stop(self):
        self.send_cmd(0.0, 0.0)

    def stop_hard(self):
        """Several zero commands, for shutdown; never raises."""
        for _ in range(5):
            try:
                self.stop()
            except Exception:                      # noqa: BLE001 - context may be gone
                return
            time.sleep(0.05)

    def wait_ready(self, timeout=30.0, need=('odom', 'camera', 'lidar')):
        t0 = time.monotonic()
        last_print = -10.0
        while True:
            missing = []
            if 'odom' in need and self.pose() is None:
                missing.append(f'odometry ({self.odom_topic})')
            if 'camera' in need and self.latest_frame() is None:
                missing.append(f'camera ({self.image_topic})')
            if 'lidar' in need and self.latest_scan() is None:
                missing.append(f'lidar ({self.scan_topic})')
            if not missing:
                break
            el = time.monotonic() - t0
            if el > timeout:
                raise MissionError(
                    'No data from ' + ', '.join(missing) + '. If the robot is on its dock, the '
                    'lidar and camera are switched off: undock first. Otherwise check the robot '
                    'profile (select_robot.bash) and `ros2 topic hz <topic>`.')
            if el - last_print >= 3.0:
                print('Waiting for ' + ', '.join(missing) + ' ...', flush=True)
                last_print = el
            time.sleep(0.1)
        # Anything received before odometry has a placeholder pose: wait for fresh data.
        t_ready = time.monotonic()
        while time.monotonic() - t_ready < 3.0:
            f, s = self.latest_frame(), self.latest_scan()
            if ('camera' not in need or f.t > t_ready) and ('lidar' not in need or s.t > t_ready):
                break
            time.sleep(0.05)
        self.pop_frames()
        parts = []
        if 'camera' in need:
            f = self.latest_frame()
            parts.append(f'camera {f.width}x{f.height}')
        if 'lidar' in need:
            parts.append(f'lidar {self.latest_scan().ranges.size} points')
        if 'odom' in need:
            parts.append('odometry')
        print('Sensors ready: ' + ', '.join(parts), flush=True)
