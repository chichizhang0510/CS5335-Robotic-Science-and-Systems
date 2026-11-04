# Homework 8 / Project 8 — Setup and Reproduction Guide

This folder contains the working scripts and mapping artifacts from this Homework 8 session:

- `scripts/`: working simulation scripts and a saved RViz configuration;
- `maps/warehouse_slam.pgm` and `maps/warehouse_slam.yaml`: the SLAM map saved from the warehouse simulation;
- this README: completed work, environment rules, and the extra-credit (real-robot Lab Map) procedure.

## Current status

**All required simulation tasks have been completed and tested.** The extra credit (Lab Map) is not done yet.

1. TurtleBot 4 Gazebo simulation: initial pose in RViz, undocking, and Nav2 Goals.
2. RViz tutorial: added a `TF` display and changed the `LaserScan` appearance (color, decay time, point size); saved as `scripts/new_config.rviz` (details in Step 3).
3. SLAM tutorial: mapped the warehouse simulation and saved the PGM + YAML map files in `maps/` (the map has some drift; see the note in Step 4).
4. Navigation tutorial: localized and navigated on the built-in warehouse map.
5. Python Navigator tutorial (**the instructor demonstration**): created a path with three poses; the robot drove a triangular route and **successfully docked at the end**.

Teammates can now reproduce the required simulation work or collect screenshots. ROS, simulation packages, and the graphics-driver setup do not need to be installed again on this laptop.

## Setup already completed on this laptop

- ROS 2 Jazzy, TurtleBot 4 simulation, and the course repository are available.
- TurtleBot 4 Python tutorials were built under `~/turtlebot4_ws`, and new terminals load them through the shell configuration.
- The NVIDIA graphics driver was repaired, so Gazebo and RViz can run.
- Canvas `.bash` scripts were converted from Windows CRLF to LF line endings (the original Canvas files are in commit `735aa5b`). The repository's `.gitattributes` keeps `*.bash` and `*.sh` files LF on every checkout, including Windows, so copying them to Ubuntu does not bring back `\r` errors.
- A `simmode` command was added for local simulation.

If a new terminal reports `simmode: command not found`, run:

```bash
source ~/.bashrc
type simmode
```

## Important: Simulation Mode (`simmode`, “C/CSI mode”)

### Why it is required

This laptop can also connect to a physical lab TurtleBot. Physical-robot communication uses ROS 2 Discovery Server settings. Gazebo, RViz, Nav2, and SLAM simulation must instead discover **only local processes**.

If simulation is started while physical-robot discovery settings are still enabled, the real-robot network can mix with local simulation. This can cause misleading symptoms: topics may appear to exist, but Gazebo or RViz waits for incorrect data; the robot, map, or correct state may not appear.

`simmode` does not start Gazebo. It switches the current terminal to the correct ROS communication environment for local simulation:

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=37
ros2 daemon stop
```

In short, it removes physical-robot connection settings and puts the terminal on the local simulation domain.

### Rules

- Run `simmode` in **every terminal used for simulation**: the terminal starting Gazebo, the terminal sending undock, and any terminal running SLAM or a tutorial command.
- Run it again in every newly opened terminal; environment variables do not transfer automatically.
- For the physical robot, use a fresh terminal with `select_robot.bash f` instead. **Do not run `simmode` for physical-robot work.**

To inspect the terminal environment:

```bash
env | grep -E 'ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT'
```

For simulation, the desired result is `ROS_DOMAIN_ID=37` only, with no `ROS_DISCOVERY_SERVER` or `ROS_SUPER_CLIENT`.

## Reproducing the simulation

### 1. Start the warehouse simulation

Terminal 1:

```bash
simmode
cd ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/hw8/scripts
bash sim.bash
```

When Gazebo and RViz open, use **2D Pose Estimate** to set the initial pose close to the dock. Wait for the global costmap before sending a goal. Do not click `Startup` in the Nav2 panel unnecessarily.

### 2. Undock and verify

The assignment says to undock **in Gazebo**: use the undock control in the Gazebo window's TurtleBot 4 HMI panel and wait for the undock to finish. If it does not respond, undock from Terminal 2 instead:

```bash
simmode
sleep 8
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" --feedback
```

Either way, check the dock status (Terminal 2, after `simmode`):

```bash
bash ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/hw8/scripts/query_dock.bash
```

The expected result is `is_docked: false`. Then select **Nav2 Goal** in RViz, choose an open area, drag to specify heading, and send the goal.

### 3. RViz assignment items

- Go to **Displays → Add → By display type → rviz_default_plugins → TF**.
- Expand `LaserScan` and change its appearance: `Style`, `Size (Pixels)` / `Size (m)`, `Decay Time`, `Color Transformer`. The `Color` field only takes effect when `Color Transformer` is `FlatColor`.
- To preserve the visual setup, use `File → Save Config As` in RViz. To reuse a saved setup, use `File → Open Config`.

What the saved `scripts/new_config.rviz` changes compared with the stock TurtleBot 4 `navigation.rviz`:

- an added, enabled `TF` display;
- `LaserScan`: `Decay Time` 0 → 2 s, `Size (Pixels)` 3 → 4, `Color` white → green (46, 194, 126).

`Style` (`Points`) and `Color Transformer` (`Intensity`) are still the defaults in that file, so the green color is not visible until `Color Transformer` is switched to `FlatColor`.

### 4. Repeat SLAM mapping if needed

Stop the current simulation using `Ctrl-C` in its launch terminal and wait for it to exit. Then run:

```bash
simmode
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py slam:=true nav2:=true rviz:=true
```

Open another terminal to undock:

```bash
simmode
sleep 8
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

Use Nav2 Goals to move progressively from known space toward the edge of unexplored space. White is generally free space, black is an obstacle/wall, and gray is unknown. Save after enough of the area has been explored:

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '/home/robo/maps/warehouse_slam'"
```

`result=0` means the save succeeded. The map saved during this session is already copied to this folder's `maps/` directory.

**Map quality note.** Overlaid on the official `turtlebot4_navigation/maps/warehouse.pgm`, the saved `maps/warehouse_slam` map shows drift. The middle section lines up, but the upper section is rotated by about 3°, the lower shelves tilt the other way, some walls are offset by 1–1.5 m, and shelf edges appear doubled. It is acceptable as the output of the mapping tutorial, but do not use it for localization or navigation. For a cleaner map, drive slowly, avoid fast spins, and return to areas that are already mapped (especially the start) before saving, so `slam_toolbox` can correct the drift.

### 5. Python Navigator: Create Path (instructor demonstration)

This is the step demonstrated for the instructor.

1. Start a **fresh** simulation as in Step 1 and wait for the costmaps. Leave the robot **on the dock**: the script undocks it itself. If the robot has already been undocked, the script first tries to dock it, which only works when the dock is close by.
2. In a new terminal:

   ```bash
   simmode
   source ~/turtlebot4_ws/install/setup.bash
   ros2 run turtlebot4_python_tutorials create_path
   ```

   `simmode` is required here too. Without it, the script runs on a different ROS domain from the simulation and waits for Nav2 forever.
3. When prompted, click and drag with **2D Pose Estimate** in RViz to add each pose of the path, then press Enter in the terminal to finish. The script then sets the initial pose to the dock (0, 0), waits for Nav2, undocks, follows the poses in order, and docks again at the end.

**Assignment requirement:** the path must have **at least two segments of 1 m or more with an angle between them**. RViz's default grid is 1 m per cell, so keep consecutive poses at least one cell apart and not on a straight line. For example, use a triangle with 1.5–2 m sides placed in open space in front of the dock. Every pose click during path creation is added to the path, so do not use 2D Pose Estimate for anything else while the script is collecting poses.

This was tested successfully: the robot followed a triangular path and docked.

### 6. Stop simulation

Press `Ctrl-C` in the terminal that started Gazebo and wait for all processes to exit. Do not only close Gazebo/RViz windows, since ROS nodes may continue running in the background.

## Extra credit: physical-robot Lab Map (10 points) — not done yet

Goal: map at least half of the lab (or a similar-sized space) with the real TurtleBot, then drive it through that area with Nav2 on the new map. The demonstration is showing the instructor the map and a Nav2 drive through the mapped area.

### What happened in the first attempt (and why)

`select_robot.bash f` discovered topics from TurtleBot F (`/scan`, `/dock_status`, `/battery_state`, `/imu`). Running only `ros2 launch turtlebot4_viz view_navigation.launch.py` then showed an empty grid, the `Map` display in `WARN`, and `Waiting for the slam_toolbox node configuration.` This is expected with that setup, for two reasons:

- **SLAM was not running.** `view_navigation` uses `map` as its fixed frame, and nothing publishes `map` until SLAM or localization is running. Without it, RViz draws nothing (no robot, no scan). The `slam_toolbox` message is the RViz SLAM panel waiting for a `slam_toolbox` node that had not been started.
- **The robot was on the dock.** TurtleBot 4's `power_saver: true` setting (the default in `turtlebot4.yaml`) turns the **lidar and camera off while docked**. So `/scan` can appear in `ros2 topic list` without publishing any data.

Fix: undock first, start SLAM, and only then open RViz.

### Before starting

- The robot is charged and on its dock in the lab, the laptop is on the lab network, there is open space around the dock, and someone is watching the robot. Also follow the safety rules in `robot_profiles/readme.md`.
- **Every terminal** below starts with the robot profile, **not** `simmode`, and nothing simulation-related (no Gazebo, no `sim.bash`) runs at the same time:

  ```bash
  source ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/robot_profiles/select_robot.bash f
  ```

  Adjust the path if the repository is cloned elsewhere, and the letter if you use a different robot.

### Part A: check the connection and undock (Terminal 1)

```bash
ros2 topic list --no-daemon --spin-time 5     # expect /scan, /odom, /tf, /cmd_vel, /dock_status
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
ros2 topic hz /scan                           # after undocking: a steady rate; Ctrl-C to stop
```

If `/scan` still has no rate after undocking, fix the connection before going further. See `robot_profiles/readme.md`.

### Part B: build the Lab Map

Terminal 2: start SLAM on the laptop. Synchronous SLAM is the manual's recommendation for a remote PC:

```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

Terminal 3: start RViz. The robot model, laser points, and a growing map should now appear:

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

Terminal 4: drive the robot slowly by keyboard (Jazzy needs `stamped:=true`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Cover **at least half of the lab**. Drive slowly, avoid spinning in place quickly, and finish by driving back near the start so `slam_toolbox` can close the loop. When the walls in RViz look straight and single (not doubled), save the map from Terminal 1:

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '/home/robo/maps/lab_map'"
```

`result=0` means success, and `~/maps/lab_map.pgm` and `~/maps/lab_map.yaml` now exist. Copy both into this folder's `maps/` directory so they are committed with the homework. Then stop SLAM with `Ctrl-C` in Terminal 2 and wait for it to exit. **Do not run SLAM and localization at the same time.**

### Part C: drive through the lab with Nav2 on the Lab Map

Terminal 2: localization on the saved map:

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/robo/maps/lab_map.yaml
```

Terminal 4 (stop teleop first): Nav2:

```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

Terminal 3: if RViz is still open from Part B, close it and start it again so it picks up the new map:

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

In RViz:

1. Use **2D Pose Estimate** to mark where the robot really is and which way it faces. Nav2 does not plan until an initial pose is set.
2. Wait for the global and local costmaps to appear and for the laser points to line up with the map walls.
3. Send a **Nav2 Goal** across the mapped lab area, then a few more goals, so the drive covers the mapped area. This is what you demonstrate to the instructor, together with the map.

When finished, send the robot back to the dock (`ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"`, or drive it close to the dock first), then stop the launch files with `Ctrl-C`.

### Extra-credit troubleshooting

- **RViz is empty, `Map` is `WARN`, or it says "Frame [map] does not exist":** SLAM (Part B) or localization (Part C) is not running yet.
- **`/scan` is listed but no laser points appear:** the robot is docked and the lidar is off (power saver). Undock, then check `ros2 topic hz /scan`.
- **Repeated `Message Filter dropping message` warnings from `slam_toolbox`:** usually a clock difference between the laptop and the robot, or a slow Wi-Fi link. Compare `date` on both machines, and try `slam.launch.py sync:=false`.
- **Teleop messages are sent but the robot does not move, or it turns by itself:** see the known per-robot issues in `robot_profiles/readme.md`, and use another robot if needed.
- Do not mix simulation commands with real-robot commands in the same terminal or at the same time.

## Quick troubleshooting

- **`simmode` does not exist:** run `source ~/.bashrc` and try again.
- **Simulation RViz has no robot or map:** stop it, open a new terminal, run `simmode` first, and restart it.
- **The Gazebo/RViz Undock control does nothing:** use the terminal action command in Step 2.
- **`create_path` waits forever for Nav2:** the terminal is missing `simmode` (Step 5).
- **SLAM navigation drives in a small loop:** the goal may be too close to a wall/obstacle or outside a safe local-costmap area. Choose a goal in more open space and save only after map coverage and wall alignment are acceptable.
