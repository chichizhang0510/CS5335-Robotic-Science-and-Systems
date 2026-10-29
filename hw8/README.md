# Homework 8 / Project 8 — Setup and Reproduction Guide

This folder contains the working scripts and mapping artifacts from this Homework 8 session:

- `scripts/`: working simulation scripts and a saved RViz configuration;
- `maps/warehouse_slam.pgm` and `maps/warehouse_slam.yaml`: the SLAM map saved from the warehouse simulation;
- this README: completed work, environment rules, and the remaining real-robot issue.

## Current status

**All required simulation tasks have been completed and tested.**

1. TurtleBot 4 Gazebo simulation: initial pose in RViz, undocking, and Nav2 Goals.
2. RViz tutorial: added a `TF` display and changed `LaserScan` size, style, and color transformer.
3. SLAM tutorial: mapped the warehouse simulation and saved the PGM + YAML map files in `maps/`.
4. Navigation tutorial: localized and navigated on an existing map.
5. Python Navigator tutorial: created a path with three poses; the robot drove a triangular route and **successfully docked at the end**.

Teammates can now reproduce the required simulation work or collect screenshots. ROS, simulation packages, and the graphics-driver setup do not need to be installed again on this laptop.

## Setup already completed on this laptop

- ROS 2 Jazzy, TurtleBot 4 simulation, and the course repository are available.
- TurtleBot 4 Python tutorials were built under `~/turtlebot4_ws`, and new terminals load them through the shell configuration.
- The NVIDIA graphics driver was repaired, so Gazebo and RViz can run.
- Canvas `.bash` scripts were fixed for Windows CRLF line endings.
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

Terminal 2:

```bash
simmode
sleep 8
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" --feedback
bash ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/hw8/scripts/query_dock.bash
```

The expected result is `is_docked: false`. Then select **Nav2 Goal** in RViz, choose an open area, drag to specify heading, and send the goal.

### 3. RViz assignment items

- Go to **Displays → Add → By display type → rviz_default_plugins → TF**.
- Expand `LaserScan` and change `Size (m)`, `Style`, and `Color Transformer`. Selecting `FlatColor` exposes a color field.
- To preserve the visual setup, use `File → Save Config As` in RViz.

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

### 5. Optional: Python Navigator triangular path

```bash
source ~/turtlebot4_ws/install/setup.bash
ros2 run turtlebot4_python_tutorials create_path
```

Follow the prompt and use **2D Pose Estimate** to add three poses. Press Enter to finish. This was tested successfully: the robot followed the triangle and docked.

### 6. Stop simulation

Press `Ctrl-C` in the terminal that started Gazebo and wait for all processes to exit. Do not only close Gazebo/RViz windows, since ROS nodes may continue running in the background.

## Extra credit: physical-robot Lab Map — not finished

The extra-credit task was started, but **real-robot mapping is not finished** and is not a completed deliverable.

This command successfully discovered some topics from physical TurtleBot F, including `/scan`, `/dock_status`, `/battery_state`, and `/imu`:

```bash
source ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/robot_profiles/select_robot.bash f
```

However, after starting:

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

RViz showed only a black/gray grid. The `Map` display was `WARN`; the physical robot, laser points, and map were not visible. The log also reported `Waiting for the slam_toolbox node configuration.`

The unresolved issue is that the physical robot's RViz / TF / scan data path is not fully established. It is separate from the completed simulation work. The robot being on its dock is not, by itself, a sufficient explanation.

To continue the extra credit:

1. Open a **new terminal** and run `select_robot.bash f`; do not run `simmode` and do not launch Gazebo.
2. Verify continuous scan data and required TF transforms (for example, `odom → base_link`).
3. Once the physical robot and `/scan` are visible in RViz, start real-robot SLAM, undock, explore the lab, and save the Lab Map.
4. Do not mix simulation commands with real-robot commands while diagnosing this connection.

## Quick troubleshooting

- **`simmode` does not exist:** run `source ~/.bashrc` and try again.
- **Simulation RViz has no robot or map:** stop it, open a new terminal, run `simmode` first, and restart it.
- **The RViz Undock button does nothing:** use the terminal action command in Step 2.
- **SLAM navigation drives in a small loop:** the goal may be too close to a wall/obstacle or outside a safe local-costmap area. Choose a goal in more open space and save only after map coverage and wall alignment are acceptable.
