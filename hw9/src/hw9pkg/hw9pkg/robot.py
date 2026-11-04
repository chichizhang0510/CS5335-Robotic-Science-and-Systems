"""
HW9: the ROS side shared by cone_finder and calibrate.

RobotIO keeps the newest camera image, lidar scan and odometry message,
and drives with closed-loop /cmd_vel (TwistStamped) commands checked
against /odom: turns stop at the requested yaw change, moves at the
requested distance along the starting heading.

ROS callbacks run in a background thread (spin_in_background), so the
main thread can block on input() or inside a motion without starving them.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan

from hw9pkg.color_detect import image_from_msg
from hw9pkg.lidar_objects import angle_step, nearest_in_sector, scan_points, wrap_angle

LOOP_DT = 0.05                       # s, motion control period (20 Hz, like /odom)
FRONT_SECTOR = math.radians(35)      # half-angle checked for obstacles while driving


class MotionError(RuntimeError):
    pass


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class RobotIO(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # Camera geometry: fx / cx from camera_info; hfov_deg only if that is missing
        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('hfov_deg', 69.0)
        # Lidar: TurtleBot 4's rplidar_link is rotated +90 deg about z (hw6 / hw7)
        self.declare_parameter('laser_yaw_offset', math.pi / 2)   # rad
        self.declare_parameter('min_valid_range', 0.12)           # m, closer = robot's own body
        # Lidar range to an object touching the front bumper. MEASURE IT:
        # `ros2 run hw9pkg calibrate lidar` with a cone touching the bumper.
        self.declare_parameter('touch_range', 0.20)               # m
        self.declare_parameter('safety_margin', 0.008)            # m, emergency stop before touching
        self.declare_parameter('max_linear_speed', 0.12)          # m/s
        self.declare_parameter('min_linear_speed', 0.03)          # m/s
        self.declare_parameter('max_angular_speed', 0.6)          # rad/s
        self.declare_parameter('min_angular_speed', 0.12)         # rad/s
        self.declare_parameter('turn_tolerance_deg', 1.0)
        self.declare_parameter('drive_tolerance', 0.004)          # m
        self.declare_parameter('settle_time', 0.4)                # s to wait after stopping

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.image_topic = gp('image_topic')
        self.scan_topic = gp('scan_topic')
        self.odom_topic = gp('odom_topic')
        self.use_info = bool(gp('use_camera_info'))
        self.hfov = math.radians(float(gp('hfov_deg')))
        self.yaw_offset = float(gp('laser_yaw_offset'))
        self.min_valid = float(gp('min_valid_range'))
        self.touch_range = float(gp('touch_range'))
        self.safety_margin = float(gp('safety_margin'))
        self.max_v = float(gp('max_linear_speed'))
        self.min_v = float(gp('min_linear_speed'))
        self.max_w = float(gp('max_angular_speed'))
        self.min_w = float(gp('min_angular_speed'))
        self.turn_tol = math.radians(float(gp('turn_tolerance_deg')))
        self.drive_tol = float(gp('drive_tolerance'))
        self.settle_time = float(gp('settle_time'))

        # newest message of each kind, with the monotonic time it arrived
        self._image = None
        self._info = None
        self._scan = None
        self._odom = None

        image_type = CompressedImage if self.image_topic.endswith('compressed') else Image
        self.create_subscription(image_type, self.image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self._cmd = self.create_publisher(TwistStamped, gp('cmd_vel_topic'), 10)

    # ------------------------------------------------------------------ #
    # Callbacks: just keep the newest message
    # ------------------------------------------------------------------ #
    def _on_image(self, msg):
        self._image = (msg, time.monotonic())

    def _on_info(self, msg):
        self._info = (msg, time.monotonic())

    def _on_scan(self, msg):
        self._scan = (msg, time.monotonic())

    def _on_odom(self, msg):
        self._odom = (msg, time.monotonic())

    # ------------------------------------------------------------------ #
    # Sensing
    # ------------------------------------------------------------------ #
    def _wait_newer(self, attr, after, timeout, what):
        """Block until the stored message `attr` arrived after `after`; return it."""
        deadline = time.monotonic() + timeout
        while True:
            item = getattr(self, attr)
            if item is not None and item[1] > after:
                return item[0]
            if time.monotonic() > deadline:
                raise TimeoutError(f'no {what} received for {timeout:.0f} s')
            time.sleep(0.02)

    def wait_ready(self, camera=True, timeout=20.0):
        """Wait for the first lidar, odometry (and camera) messages."""
        need = [('_scan', self.scan_topic), ('_odom', self.odom_topic)]
        if camera:
            need.append(('_image', self.image_topic))
        self.get_logger().info('Waiting for ' + ', '.join(t for _, t in need) + ' ...')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(getattr(self, a) is not None for a, _ in need):
                if camera and self._info is None and self.use_info:
                    self.get_logger().warn('No camera_info yet: using hfov_deg for pixel angles.')
                return
            time.sleep(0.1)
        missing = [t for a, t in need if getattr(self, a) is None]
        raise RuntimeError(
            'No messages on ' + ', '.join(missing) + '. Check that you sourced '
            'robot_profiles/select_robot.bash for this robot and that '
            '`ros2 topic hz <topic>` shows data.')

    def frame(self, timeout=3.0):
        """A camera image (BGR array) received after this call."""
        msg = self._wait_newer('_image', time.monotonic(), timeout, f'image on {self.image_topic}')
        return image_from_msg(msg)

    def latest_frame(self):
        """Newest camera image without waiting, or None."""
        return image_from_msg(self._image[0]) if self._image else None

    def points(self, timeout=3.0):
        """(points, angle step) from a lidar scan received after this call."""
        scan = self._wait_newer('_scan', time.monotonic(), timeout, f'scan on {self.scan_topic}')
        return scan_points(scan, self.yaw_offset, self.min_valid), angle_step(scan)

    def pose(self):
        """(x, y, yaw) from the newest /odom message."""
        if self._odom is None:
            raise MotionError('no odometry yet')
        p = self._odom[0].pose.pose
        return p.position.x, p.position.y, yaw_from_quaternion(p.orientation)

    def camera_bearing(self, u, width):
        """Robot-frame angle (rad, + = left) of image column u."""
        info = self._info[0] if (self._info and self.use_info) else None
        if info is not None and info.k[0] > 0 and info.width > 0:
            scale = width / info.width
            fx, cx = info.k[0] * scale, info.k[2] * scale
        else:
            fx = (width / 2.0) / math.tan(self.hfov / 2.0)
            cx = (width - 1) / 2.0
        return math.atan2(cx - u, fx)

    def front_clearance(self):
        """Nearest lidar range ahead (within FRONT_SECTOR) in the newest scan."""
        if self._scan is None or time.monotonic() - self._scan[1] > 1.0:
            raise MotionError('lidar stopped publishing; stopping for safety')
        pts = scan_points(self._scan[0], self.yaw_offset, self.min_valid)
        return nearest_in_sector(pts, FRONT_SECTOR)

    # ------------------------------------------------------------------ #
    # Motion (closed loop on /odom)
    # ------------------------------------------------------------------ #
    def publish_twist(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd.publish(msg)

    def stop(self):
        for _ in range(3):
            self.publish_twist(0.0, 0.0)
            time.sleep(0.02)

    def settle(self):
        time.sleep(self.settle_time)

    def _check_odom(self):
        if self._odom is None or time.monotonic() - self._odom[1] > 1.0:
            raise MotionError('odometry stopped publishing; stopping')

    def rotate_by(self, angle):
        """Turn in place by `angle` rad (+ = left / counter-clockwise)."""
        if abs(angle) <= self.turn_tol:
            return
        _, _, last = self.pose()
        turned = 0.0
        deadline = time.monotonic() + abs(angle) / self.min_w + 5.0
        try:
            while True:
                self._check_odom()
                _, _, yaw = self.pose()
                turned += wrap_angle(yaw - last)     # unwrap: total turned so far
                last = yaw
                remaining = angle - turned
                if abs(remaining) <= self.turn_tol:
                    break
                if time.monotonic() > deadline:
                    raise MotionError(f'turn of {math.degrees(angle):.1f} deg timed out')
                speed = clamp(1.5 * abs(remaining), self.min_w, self.max_w)
                self.publish_twist(0.0, math.copysign(speed, remaining))
                time.sleep(LOOP_DT)
        finally:
            self.stop()
        self.settle()

    def drive_by(self, distance, check_front=True):
        """Drive straight `distance` m (+ = forward), holding the start heading.

        Going forward, stops early if anything comes within
        touch_range + safety_margin of the lidar. Returns the distance
        actually driven according to odometry.
        """
        if abs(distance) <= self.drive_tol:
            return 0.0
        x0, y0, yaw0 = self.pose()
        c, s = math.cos(yaw0), math.sin(yaw0)
        done = 0.0
        deadline = time.monotonic() + abs(distance) / self.min_v + 5.0
        try:
            while True:
                self._check_odom()
                x, y, yaw = self.pose()
                done = (x - x0) * c + (y - y0) * s      # progress along the start heading
                remaining = distance - done
                if abs(remaining) <= self.drive_tol:
                    break
                if time.monotonic() > deadline:
                    raise MotionError(f'move of {distance:.3f} m timed out')
                if check_front and remaining > 0:
                    clear = self.front_clearance()
                    if clear is not None and clear < self.touch_range + self.safety_margin:
                        self.get_logger().warn(
                            f'Safety stop: something {clear:.3f} m ahead of the lidar.')
                        break
                speed = clamp(1.2 * abs(remaining), self.min_v, self.max_v)
                steer = clamp(2.0 * wrap_angle(yaw0 - yaw), -0.3, 0.3)
                self.publish_twist(math.copysign(speed, remaining), steer)
                time.sleep(LOOP_DT)
        finally:
            self.stop()
        self.settle()
        return done


def spin_in_background(node):
    """Run the node's callbacks in a daemon thread; returns the executor."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def run():
        try:
            executor.spin()
        except (KeyboardInterrupt, Exception):   # Ctrl-C / shutdown ends the thread
            pass

    threading.Thread(target=run, daemon=True).start()
    return executor


def shutdown(node, executor):
    """Stop the robot and tear everything down; safe to call after Ctrl-C."""
    try:
        node.stop()
    except Exception:
        pass
    try:
        executor.shutdown(timeout_sec=1.0)
    except Exception:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
