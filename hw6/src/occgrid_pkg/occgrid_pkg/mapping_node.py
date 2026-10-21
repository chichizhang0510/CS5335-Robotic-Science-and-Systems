"""
ROS2 occupancy-grid mapping node (Task 1 + Task 2 on the real robot).

- Subscribes to LaserScan (/scan) and Odometry (/odom)
- The first odom message defines the grid origin (robot start = cell (100, 100))
- Updates the grid at start and then every `update_distance` (0.1 m) of travel
- Saves PNG snapshots at `snapshot_distances` (0.0, 0.5, 1.0 m) + one per update
- Ctrl+C: one final update with the latest scan, saves grid_final.png

Run:
  ros2 run occgrid_pkg mapping_node
  ros2 run occgrid_pkg mapping_node --ros-args -p laser_yaw_offset:=0.0 -p scan_topic:=/scan
Then drive 1 m with your existing Homework 4 ``move_turn`` action or the
Turtlebot's built-in ``/drive_distance`` action in another terminal.
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from occgrid_pkg.grid_core import OccupancyGrid, save_grid_image


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MappingNode(Node):
    def __init__(self):
        super().__init__('occgrid_mapping_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        # TurtleBot 4's rplidar_link is mounted rotated +90 deg about z relative
        # to base_link, so scan angle 0 points to the robot's left, not forward.
        self.declare_parameter('laser_yaw_offset', math.pi / 2)  # rad, lidar vs base_link
        self.declare_parameter('update_distance', 0.1)        # m
        self.declare_parameter('snapshot_distances', [0.0, 0.5, 1.0])
        # Keep this at zero: the assignment asks for maps after each 0.1 m,
        # not before the robot reaches that distance.  A tiny floating-point
        # comparison error cannot make a ROS odometry message exactly 0.1 m,
        # so using zero still triggers on the first message at/after a mark.
        self.declare_parameter('tolerance', 0.0)                # m, never trigger early
        self.declare_parameter('save_every_update', True)
        # A relative path is resolved from the terminal's current folder.
        # When launched from hw6, this keeps all generated images in
        # hw6/occgrid_output instead of cluttering the home directory.
        self.declare_parameter('output_dir', './occgrid_output')

        gp = self.get_parameter
        self.scan_topic = gp('scan_topic').value
        self.odom_topic = gp('odom_topic').value
        self.yaw_offset = float(gp('laser_yaw_offset').value)
        self.update_dist = float(gp('update_distance').value)
        self.pending_snaps = sorted(float(s) for s in gp('snapshot_distances').value)
        self.tol = float(gp('tolerance').value)
        self.save_every = bool(gp('save_every_update').value)
        self.out_dir = os.path.expanduser(gp('output_dir').value)
        os.makedirs(self.out_dir, exist_ok=True)

        self.grid = OccupancyGrid()
        self.odom0 = None          # (x, y, yaw) of the first odom message
        self.pose = None           # robot pose relative to the start
        self.latest_scan = None
        self.next_mark = 0         # next update at next_mark * update_dist
        self.n_updates = 0
        self.last_update_dist = None

        # sensor-data QoS (BEST_EFFORT) - Turtlebot 4 /scan is best effort;
        # a best-effort subscriber also connects to a reliable /odom publisher
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            f'Listening on {self.scan_topic} and {self.odom_topic}; '
            f'images -> {self.out_dir}')

    # ------------------------------------------------------------------ #
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.odom0 is None:
            self.odom0 = (p.x, p.y, yaw)
            self.get_logger().info(
                f'Grid origin set at odom ({p.x:.3f}, {p.y:.3f}, yaw {yaw:.3f})')
        x0, y0, th0 = self.odom0
        dx, dy = p.x - x0, p.y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        # express pose in the start frame (robot initially faces +x)
        self.pose = (c * dx - s * dy, s * dx + c * dy, wrap_angle(yaw - th0))

    def scan_cb(self, msg):
        self.latest_scan = msg
        if self.pose is None:
            return                                    # wait for odometry
        dist = math.hypot(self.pose[0], self.pose[1])
        if dist >= self.next_mark * self.update_dist - self.tol:
            self.do_update(msg, dist)
            self.next_mark = int(math.floor((dist + self.tol) / self.update_dist)) + 1

    # ------------------------------------------------------------------ #
    def do_update(self, msg, dist, tag=None):
        pose = self.pose
        self.grid.update(list(msg.ranges), msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max, pose, self.yaw_offset)
        self.n_updates += 1
        self.last_update_dist = dist
        self.get_logger().info(
            f'Update #{self.n_updates} at {dist:.3f} m, pose '
            f'({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg)')

        if self.save_every:
            self.save(f'update_{self.n_updates:02d}_{dist:.2f}m.png', dist, pose)

        while self.pending_snaps and dist >= self.pending_snaps[0] - self.tol:
            s = self.pending_snaps.pop(0)
            self.save(f'grid_{s:.1f}m.png', dist, pose,
                      f'Snapshot {s:.1f} m (actual {dist:.3f} m)')

        if tag:
            self.save(f'grid_{tag}.png', dist, pose, f'Final ({dist:.3f} m)')

    def save(self, name, dist, pose, title=None):
        path = os.path.join(self.out_dir, name)
        save_grid_image(self.grid, path, pose,
                        title or f'{self.n_updates} updates, {dist:.2f} m')
        self.get_logger().info(f'Saved {path}')

    def finalize(self):
        """Called on Ctrl+C: integrate the latest scan once more and save."""
        if self.latest_scan is not None and self.pose is not None:
            dist = math.hypot(self.pose[0], self.pose[1])
            if self.last_update_dist is None or abs(dist - self.last_update_dist) > 0.02:
                self.do_update(self.latest_scan, dist, tag='final')
            else:   # robot hasn't moved since the last update: don't count the same view twice
                self.save('grid_final.png', dist, self.pose, f'Final ({dist:.3f} m)')


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
