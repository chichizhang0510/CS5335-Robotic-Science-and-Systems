# Homework 7 — Two-way mapping, landmarks and A*

The `hw7pkg` package reuses the hw6 occupancy grid (5 cm cells, log-odds,
Bresenham ray casting, lidar mounted at +90°). The default grid is
400 × 400 cells, which covers 20 m × 20 m, so a 3 m path plus lidar range
fits. It contains:

| Program | Task | What it does |
|---|---|---|
| `mapping_node` | 1 | ROS 2 node. Builds one grid per run, updating after every 0.1 m travelled along the path, and saves `<run_name>.npz` + `<run_name>_final.png` |
| `landmarks` | 2 | Finds corner and post landmarks in two grids, matches them, transforms run 2 into run 1's frame and writes the report and images |
| `plan_path` | 3 | A* on a grid, with obstacles grown by the TurtleBot's radius. Draws the path and saves `turn`/`move` commands to replay with the hw4 executive |
| `simulate` | — | Writes synthetic `run1.npz` / `run2.npz` (with odometry drift) to try Tasks 2 and 3 without the robot |

## Setup (every terminal)

```bash
cd ~/CS5335-Robotic-Science-and-Systems
source robot_profiles/select_robot.bash f
```

## Build

```bash
cd hw7
colcon build --packages-select hw7pkg
source install/setup.bash
```

All commands below are run from `hw7`, so output goes to `hw7/hw7_output/`.

## Task 1 — map out, turn around, map back

Terminal 2, from the repo root, the hw4 driving nodes (leave running):

```bash
source hw4/install/setup.bash
ros2 launch day4pkg day4.launch.py
```

**Outbound run.** Terminal 1, from `hw7`:

```bash
source install/setup.bash
ros2 run hw7pkg mapping_node --ros-args -p run_name:=run1
```

Wait for `update #1`, then type in terminal 2:

```
move 3.2
```

When the robot stops, press Ctrl-C in terminal 1. This saves `run1.npz` and
`run1_final.png`. The title shows the distance travelled and how far the
robot ended from the start. The task needs at least 3 m travelled and at
least 2 m from the start.

For a curved path, use `arc left 3.0 62` instead. That is 3.25 m along the
arc, ending 3.1 m from the start.

**Turn around.** In terminal 2:

```
turn 180
```

**Return run.** Terminal 1, start a new grid (its origin is the robot's pose
after the turn):

```bash
ros2 run hw7pkg mapping_node --ros-args -p run_name:=run2
```

Wait for `update #1`, then type `move 3.2` in terminal 2 (`arc right 3.0 62`
after a left arc). When the robot stops, press Ctrl-C in terminal 1.

Keep the area still between the two runs. People walking past show up as
landmarks in only one grid.

## Task 2 — landmarks

```bash
ros2 run hw7pkg landmarks hw7_output/run1.npz hw7_output/run2.npz
```

This prints the report and saves it as `hw7_output/report.txt`. It contains:

- the transform from run 2's frame to run 1's frame, estimated from the
  landmarks, next to the one odometry predicts
- every landmark found in both grids, with its run 1 position, the run 2
  position after the transform, and the difference
- **mean position difference** of the matched landmarks (also RMS and max),
  plus the same pairs aligned with odometry alone, for comparison
- landmarks found in run 1 only and in run 2 only

Images to show the instructor:

- `landmarks_run1.png` — run 1 grid with wall segments and landmarks A1, A2, …
- `landmarks_run2.png` — run 2 grid, in its own frame, with landmarks B1, B2, …
  (green = found in both grids, red = only in this grid)
- `overlay.png` — run 2's occupied cells and landmarks transformed onto run 1

`landmarks.csv` lists every landmark in its own frame and in run 1's frame.

**How it works.** Cells with log-odds above 1.0 count as occupied.

- **Posts:** small blobs of occupied cells (≤ 0.45 m across), such as chair
  legs, bins or narrow boxes. Each post is located at the blob's centroid.
- **Corners:** sequential RANSAC fits wall lines to the remaining cells.
  Where two wall segments meet at 45–135°, their intersection is a corner.

Run 2 is rotated about 180° from run 1. Every pair of run 1 landmarks with
the same spacing as a pair of run 2 landmarks proposes a rotation and shift
(the hint's "align one landmark, then rotate"). The proposal that puts the
most run 2 landmarks onto run 1 landmarks of the same type wins. It is then
refined by least squares on all matched pairs.

Proposals more than 1 m / 30° from odometry are ignored, so a repetitive
room cannot produce a false match. Add `--no-odom-gate` to rely on landmarks
alone.

A post's centroid is the middle of the faces the lidar saw. The return run
sees the other side of each object, so a large object's centroid shifts
between runs. This is part of the position difference.

If too few landmarks are found:

- `--occ-thresh 0.6` counts cells hit fewer times as occupied
- `--min-len 0.25` keeps shorter wall segments
- `--match-tol 0.4` accepts landmarks further apart as the same one

## Task 3 — A* around an obstacle

Task 3 only asks for the planned path drawn on an occupancy grid. The robot
stays at its start position, and terminal 2 is not needed unless you also
want to drive the path (see "Test the path" below).

**1. Set up.** Put the robot at its start position. Place an obstacle about
1 m in front of it, on the straight line to the goal (about 2.5 m straight
ahead).

**2. Take one scan from the start.** Terminal 1, from `hw7`:

```bash
source install/setup.bash
ros2 run hw7pkg mapping_node --ros-args -p run_name:=task3
```

Wait for `update #1`, then press Ctrl-C. This saves `hw7_output/task3.npz`.

**3. Plan the path.** Same terminal. The goal is in metres from the start
(+x = the way the robot faces, +y = its left):

```bash
ros2 run hw7pkg plan_path hw7_output/task3.npz --goal 2.5 0.0 --unknown free
```

`--unknown free` is needed because the area behind the obstacle was never
seen from a single spot, and the goal is in that blind area. Alternatively,
pick a goal off to the side so that the obstacle still blocks the straight
line, e.g. obstacle at (1.2, 0.3), goal at (2.5, 0.6), and drop the flag.

**4. Show the instructor** `hw7_output/astar_path.png`. It shows:

- the obstacles in dark red
- in pink, the area the robot's centre must avoid: obstacles grown by the
  0.171 m robot radius + 0.05 m margin, which is how the robot's width is
  accounted for
- the A* path, with the robot footprint drawn along it
- the shortened waypoint path

The terminal (and `astar_path.txt`) must say
`Straight line start -> goal: BLOCKED, so the path has to bend`. If it says
CLEAR, the obstacle is not on the line. Move the obstacle and repeat steps
2–3.

Useful options:

- `--margin 0.1` keeps more distance from obstacles
- `--clearance-weight 0` plans the pure shortest path
- `--start X Y --start-yaw DEG` plans from somewhere other than the origin

### Test the path (optional)

`plan_path` also saves the `turn` / `move` commands alone to
`hw7_output/astar_commands.txt`, ending with `quit`. Turns are
counter-clockwise degrees: `turn 349.2` turns 10.8° clockwise. For example:

```
turn 349.2
move 1.20
turn 302.4
move 1.16
quit
```

The hw4 executive can replay this file. `day4.launch.py` forces the executive
to read from the keyboard, so start the two hw4 nodes separately instead.
Put the robot back at the start pose, facing the same way, first.

Terminal 2, from the repo root, the driving node:

```bash
source hw4/install/setup.bash
ros2 run day4pkg driving_node
```

Terminal 3, from the repo root, the executive with the saved commands:

```bash
source hw4/install/setup.bash
ros2 run day4pkg executive_node < hw7/hw7_output/astar_commands.txt
```

The executive sends one command at a time, waits for each to finish, and
exits after `quit`. The hw4 driving node is open loop (timed), so expect the
robot to drift a little from the plan, more so after the turns.

## Try it without the robot

```bash
ros2 run hw7pkg simulate                          # straight 3.2 m out and back
ros2 run hw7pkg simulate --curve 0.25 --length 3.5   # curved path
ros2 run hw7pkg landmarks hw7_output/run1.npz hw7_output/run2.npz
ros2 run hw7pkg plan_path hw7_output/run1.npz --goal 1.6 -1.3
```

The simulated room has a small box near the start, so the last command plans
around it. The simulator overwrites `hw7_output/run1.npz` and `run2.npz`, so
run it before or in a different `--out` folder from the real runs.

## Mapping node parameters

| Parameter | Default | Meaning |
|---|---|---|
| `run_name` | `run1` | Prefix of the output files |
| `laser_yaw_offset` | `π/2` | Lidar yaw relative to `base_link` (rad); TurtleBot 4 = +90° |
| `update_distance` | `0.1` | Distance along the path between updates (m) |
| `grid_size` | `400` | Cells per side |
| `resolution` | `0.05` | Cell size (m) |
| `save_every_update` | `false` | Also save a PNG after every update |
| `output_dir` | `./hw7_output` | Folder for `.npz` and `.png` files |
| `scan_topic` / `odom_topic` | `/scan` / `/odom` | Input topics |

The `.npz` file is rewritten after every update, so a crash loses at most
one update. It holds the log-odds grid, the poses at each update, the full
trajectory and the run's odometry origin.
