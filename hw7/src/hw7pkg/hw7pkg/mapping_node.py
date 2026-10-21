"""
HW7 Task 1: build one occupancy grid per run.

Start this node once for the outbound run and once for the return run:

  ros2 run hw7pkg mapping_node --ros-args -p run_name:=run1
  ros2 run hw7pkg mapping_node --ros-args -p run_name:=run2

- The first odom message defines that run's grid origin (robot pose = (0, 0, 0)),
  so each run has its own coordinate frame, as Task 2 expects.
- The grid is updated at the start and after every `update_distance` (0.1 m)
  of distance travelled *along the path* (summed odometry steps), so curved
  paths are handled too. hw6 used straight-line distance from the start.
- After every update the grid is written to <output_dir>/<run_name>.npz,
  together with the trajectory and the absolute odom pose of the origin.
  Task 2 uses the origins to compare its landmark-based alignment with the
  alignment that odometry alone predicts.
- Ctrl+C: one final update with the latest scan, then <run_name>_final.png.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from hw7pkg.grid_core import OccupancyGrid, save_grid_image


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MappingNode(Node):
    def __init__(self):
        super().__init__('hw7_mapping_node')

        self.declare_parameter('run_name', 'run1')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        # TurtleBot 4's rplidar_link is mounted rotated +90 deg about z relative
        # to base_link, so scan angle 0 points to the robot's left (see hw6).
        self.declare_parameter('laser_yaw_offset', math.pi / 2)  # rad
        self.declare_parameter('update_distance', 0.1)           # m along the path
        self.declare_parameter('grid_size', 400)                 # cells per side
        self.declare_parameter('resolution', 0.05)               # m per cell
        self.declare_parameter('save_every_update', False)       # PNG per update
        self.declare_parameter('output_dir', './hw7_output')

        gp = self.get_parameter
        self.run_name = gp('run_name').value
        self.scan_topic = gp('scan_topic').value
        self.odom_topic = gp('odom_topic').value
        self.yaw_offset = float(gp('laser_yaw_offset').value)
        self.update_dist = float(gp('update_distance').value)
        self.save_every = bool(gp('save_every_update').value)
        self.out_dir = os.path.expanduser(gp('output_dir').value)
        os.makedirs(self.out_dir, exist_ok=True)

        self.grid = OccupancyGrid(size=int(gp('grid_size').value),
                                  resolution=float(gp('resolution').value))
        self.odom0 = None          # absolute odom (x, y, yaw) of this run's origin
        self.pose = None           # robot pose in this run's frame
        self.last_xy = None        # previous odom position, for path length
        self.travelled = 0.0       # distance along the path (m)
        self.latest_scan = None
        self.next_mark = 0         # next update at next_mark * update_dist
        self.n_updates = 0
        self.last_update_dist = None
        self.update_poses = []     # (x, y, theta, travelled) at each update
        self.trajectory = []       # every odom pose, for drawing the path

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            f'[{self.run_name}] listening on {self.scan_topic} and {self.odom_topic}; '
            f'output -> {self.out_dir}')

    # ------------------------------------------------------------------ #
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.odom0 is None:
            self.odom0 = (p.x, p.y, yaw)
            self.last_xy = (p.x, p.y)
            self.get_logger().info(
                f'Grid origin set at odom ({p.x:.3f}, {p.y:.3f}, yaw {yaw:.3f})')
        self.travelled += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

        x0, y0, th0 = self.odom0
        dx, dy = p.x - x0, p.y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        # express pose in this run's start frame (robot initially faces +x)
        self.pose = (c * dx - s * dy, s * dx + c * dy, wrap_angle(yaw - th0))
        if not self.trajectory or math.hypot(self.pose[0] - self.trajectory[-1][0],
                                             self.pose[1] - self.trajectory[-1][1]) > 0.01:
            self.trajectory.append(self.pose)

    def scan_cb(self, msg):
        self.latest_scan = msg
        if self.pose is None:
            return                                    # wait for odometry
        if self.travelled >= self.next_mark * self.update_dist:
            self.do_update(msg)
            self.next_mark = int(math.floor(self.travelled / self.update_dist)) + 1

    # ------------------------------------------------------------------ #
    def do_update(self, msg):
        pose, dist = self.pose, self.travelled
        self.grid.update(list(msg.ranges), msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max, pose, self.yaw_offset)
        self.n_updates += 1
        self.last_update_dist = dist
        self.update_poses.append((*pose, dist))
        self.get_logger().info(
            f'[{self.run_name}] update #{self.n_updates} at {dist:.3f} m travelled, pose '
            f'({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg)')

        self.save_npz()
        if self.save_every:
            self.save_png(f'{self.run_name}_update_{self.n_updates:02d}_{dist:.2f}m.png')

    def save_npz(self):
        path = os.path.join(self.out_dir, f'{self.run_name}.npz')
        self.grid.save(path,
                       odom_origin=np.array(self.odom0),
                       update_poses=np.array(self.update_poses),
                       trajectory=np.array(self.trajectory),
                       laser_yaw_offset=self.yaw_offset)

    def save_png(self, name, title=None):
        path = os.path.join(self.out_dir, name)
        save_grid_image(self.grid, path, self.pose,
                        title or f'{self.run_name}: {self.n_updates} updates, '
                                 f'{self.travelled:.2f} m travelled',
                        trajectory=self.trajectory)
        self.get_logger().info(f'Saved {path}')

    def finalize(self):
        """Called on Ctrl+C: integrate the latest scan once more and save."""
        if self.latest_scan is None or self.pose is None:
            return
        if self.last_update_dist is None or abs(self.travelled - self.last_update_dist) > 0.02:
            self.do_update(self.latest_scan)
        else:   # robot hasn't moved since the last update: don't count the same view twice
            self.save_npz()
        end = math.hypot(self.pose[0], self.pose[1])
        self.save_png(f'{self.run_name}_final.png',
                      f'{self.run_name}: {self.travelled:.2f} m travelled, '
                      f'ends {end:.2f} m from start, {self.n_updates} updates')


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finalize()
        except Exception as e:                      # logger may be gone on shutdown
            print(f'finalize failed: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
