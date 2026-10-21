# Homework 6 — Occupancy grid mapping

The `occgrid_pkg` package builds a 200 × 200 log-odds occupancy grid with
5 cm cells, which covers 10 m × 10 m. The robot's start position is the
centre of the grid. It contains:

- `grid_core.py` — the grid itself: Bresenham ray casting, the inverse
  sensor model and PNG output. It uses only NumPy, with no ROS.
- `mapping_node.py` — the ROS 2 node. It subscribes to `/scan` and `/odom`
  and updates the grid at the start and then after every 0.1 m of travel.
- `offline_test.py` — runs the same grid code on a saved scan, or on a
  synthetic room, without a robot.

## Setup (every terminal)

```bash
cd ~/CS5335-Robotic-Science-and-Systems
source robot_profiles/select_robot.bash f
```

## Build

```bash
cd hw6
colcon build --packages-select occgrid_pkg
source install/setup.bash
```

## Task 1 — single scan

Terminal 1, from `hw6`, the mapping node:

```bash
source install/setup.bash
ros2 run occgrid_pkg mapping_node
```

The first scan is integrated straight away and saved as `grid_0.0m.png`.
Press Ctrl-C to stop the node.

## Task 2 — map while driving 1 m

Terminal 1, from `hw6`, start the mapping node as in Task 1.

Terminal 2, from the repo root, the hw4 driving nodes:

```bash
source hw4/install/setup.bash
ros2 launch day4pkg day4.launch.py
```

At the executive prompt in terminal 2, type `move 1.0`. Once the robot stops,
press Ctrl-C in terminal 1. The node then integrates the latest scan one more
time and saves `grid_final.png`.

To drive with the built-in action instead of the hw4 nodes:

```bash
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance \
  "{distance: 1.0, max_translation_speed: 0.10}"
```

## Output

Images are written to `hw6/occgrid_output/` (relative to the folder the node
was started from):

- `update_NN_D.DDm.png` — the grid after each update
- `grid_0.0m.png`, `grid_0.5m.png`, `grid_1.0m.png` — snapshots at those
  distances
- `grid_final.png` — the grid after Ctrl-C

The file names repeat between runs. Before each new run, move the previous
images into a dated subfolder, such as `occgrid_output/2026.10.21.3/`.

In the images, white is free, black is occupied and gray is unknown. The red
arrow shows the robot's pose.

## Parameters

Set these with `--ros-args -p name:=value`, for example:

```bash
ros2 run occgrid_pkg mapping_node --ros-args -p update_distance:=0.2
```

| Parameter | Default | Meaning |
|---|---|---|
| `scan_topic` | `/scan` | LaserScan topic |
| `odom_topic` | `/odom` | Odometry topic |
| `laser_yaw_offset` | `π/2` | Yaw of the lidar relative to `base_link` (rad) |
| `update_distance` | `0.1` | Distance travelled between updates (m) |
| `snapshot_distances` | `[0.0, 0.5, 1.0]` | Distances at which snapshots are saved (m) |
| `save_every_update` | `true` | Save a PNG after every update |
| `output_dir` | `./occgrid_output` | Folder for the images |

The TurtleBot 4 lidar is mounted rotated +90° relative to `base_link`, so
scan angle 0 points to the robot's left. Leave `laser_yaw_offset` at `π/2`
for TurtleBot 4. With `0.0`, the map comes out rotated 90° and obstacles in
front of the robot appear to its side. To check the mounting on a robot:

```bash
ros2 topic echo -n 1 /scan --field header.frame_id
ros2 run tf2_ros tf2_echo base_link rplidar_link
```

## Offline test (no robot)

```bash
cd hw6/src/occgrid_pkg/occgrid_pkg
python3 offline_test.py                     # synthetic room, single scan
python3 offline_test.py --move              # synthetic 1 m drive
ros2 topic echo -n 1 /scan > scan.yaml      # capture a real scan on the robot
python3 offline_test.py scan.yaml --yaw-offset 1.5708
```

Add `--no-show` to save the PNGs without opening a window, for example over
SSH.
