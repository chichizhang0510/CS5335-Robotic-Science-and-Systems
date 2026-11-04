# Homework 9 / Project 9 — Find three coloured cones, then drive to one

The `hw9pkg` package does both tasks in one program, `cone_hunter`:

1. **Task 1:** the robot turns 360° and reports each cone's colour name,
   direction and distance.
2. **Task 2:** in a loop, it asks for a colour, drives to that cone and stops
   about an inch short of it without touching it.

| Program | What it does |
|---|---|
| `cone_hunter` | Tasks 1 and 2 on the real robot (the demo) |
| `cone_calib` | Calibration helper. It measures the cones' HSV colours and the stopping distance. It never moves the robot |
| `sim_test` | Offline test of the same mission code in a simulated room. Needs no robot and no ROS network |

## How it works

- **Camera (OAK-D preview, 250×250):** each colour is an HSV box. The largest
  blob of each colour is that cone. Blob pieces split by the cone's white band
  are merged.
- **Lidar:** the camera picks *which* object is which colour, and the lidar
  gives *where* it is. During the turn, every time a cone passes near the
  image centre, the lidar cluster in that direction gives its range and exact
  bearing. Clusters that are part of a wall are rejected.
- **Odometry:** the turn is closed-loop on odometry, not timed. Cone positions
  are kept in the odometry frame. Each image and scan is matched to the robot
  pose at its own time stamp, so turning while data is in flight does not blur
  the bearings.
- **Task 2:**
  1. Turn toward the stored position of the requested cone, and spin to search
     for it if it isn't in view.
  2. Centre it in the camera image.
  3. Drive at it while tracking its lidar cluster. Speed drops as the cone gets
     close, to 2.5 cm/s for the last few centimetres.
  4. Stop when the lidar range to the cone reaches `contact_range + target_gap`
     (target gap 2.5 cm = 1 inch).

  The bumper is monitored: a press stops the robot at once. Before turning to
  the next cone, the robot reverses 8 cm.

## Before the lab: build and run the offline test

All commands run from `hw9` on the uni Ubuntu laptop. OpenCV comes with ROS
Jazzy desktop. Check it first:

```bash
python3 -c "import cv2; print(cv2.__version__)"
cd ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/hw9
colcon build --packages-select hw9pkg
source install/setup.bash
ros2 run hw9pkg sim_test
```

`sim_test` simulates the following:

- the camera, rendering each cone, so the real colour detector runs on every
  image
- the lidar
- odometry with a 2% gyro error
- camera and lidar delays

It runs Task 1 and then drives to the cones in the order orange, blue, green,
orange. It ends with `PASSED` when every true bumper-to-cone gap is 1–4 cm and
nothing was touched. Images go to `hw9_output/sim/`.

You can try harder cases:

```bash
ros2 run hw9pkg sim_test --cam-hz 4 --cam-latency 0.4      # slow, laggy camera
ros2 run hw9pkg sim_test --true-hfov 69 --hfov-error -27   # wrong camera_hfov_deg
ros2 run hw9pkg sim_test --colors red yellow purple --order purple red yellow --radius 1.4
ros2 run hw9pkg sim_test --lidar-blind                     # cones too short for the lidar
```

## In the lab

### Every terminal: robot profile

```bash
source ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/robot_profiles/select_robot.bash f
cd ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems/hw9
source install/setup.bash
```

Do **not** run `simmode` (that is for Gazebo only).

### Step 0 — undock

While the robot is docked, the TurtleBot 4 switches the **lidar and camera
off**, and both programs wait for data that never comes. Undock first:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
ros2 topic hz /scan                           # steady rate; Ctrl-C
ros2 topic hz /oakd/rgb/preview/image_raw     # steady rate; Ctrl-C
```

### Step 1 — calibrate (about 5 minutes, once per set of cones)

```bash
ros2 run hw9pkg cone_calib --ros-args --params-file config/hw9_params.yaml
```

It prints a status block once a second. It also keeps
`hw9_output/calib_latest.png` up to date: the camera view next to the mask of
each colour. Open it with `eog hw9_output/calib_latest.png`, which reloads
when the file changes. Press Enter to keep a numbered copy.

**a. Colours.** Put one cone ~0.5 m straight in front of the robot, so it
covers the small box at the image centre. Read the line
`centre patch H=.. S=.. V=.. -> suggested box [..]`. Copy that box into
`config/hw9_params.yaml` under `hsv:`, using your colour name. Do the same for
each cone and set `colors:` to your three names. Restart `cone_calib` and check
the masks:

- each cone shows up white in its own mask and nowhere else
- the floor and walls stay black

If two colours overlap, the tool prints a WARNING. In that case narrow the hue
ranges (the first two numbers).

**b. The lidar must see the cones.** With a cone ~0.5 m straight ahead, the
line `lidar: nearest within 10 deg of straight ahead` must show about 0.5 m.
If it shows the wall behind instead, the cone is lower than the lidar (about
19 cm above the floor). Use taller cones; see Troubleshooting.

**c. Stopping distance.** Slide a cone toward the front of the robot until it
**just touches** the bumper. If the base is square, point a corner at the
robot, because the corner sticks out furthest. Read the lidar range on the
`nearest within 10 deg` line and set it as `contact_range` in the params file.
The robot stops at `contact_range + target_gap`, i.e. 2.5 cm short of that
contact point.

### Step 2 — the demo (Tasks 1 and 2)

Place the three cones in a triangle at least 1.5 m apart, with nothing inside
it, and the robot roughly in the middle. Keep other objects of the cone colours
out of view: clothing, posters, bags.

```bash
ros2 run hw9pkg cone_hunter --ros-args --params-file config/hw9_params.yaml
```

Task 1 prints each cone as it is spotted, then a table (this one is from
`sim_test`):

```
=== Task 1 result: found 3 of 3 cones ===
  colour    direction           distance   position (x ahead, y left)
  orange    15 deg left          0.95 m    (+0.91, +0.25) m   [68 sightings]
  green     143 deg left         0.95 m    (-0.75, +0.57) m   [42 sightings]
  blue      100 deg right        0.95 m    (-0.16, -0.93) m   [41 sightings]
  cone spacing: orange-green 1.70 m, orange-blue 1.60 m, green-blue 1.61 m
```

Directions and positions are relative to the robot's starting pose. Then Task 2
asks (again from `sim_test`, where `contact_range` is 0.258 m):

```
Which cone should I drive to? (orange, green, blue; q = quit): blue
--> Driving to the BLUE cone
  turning toward where the blue cone was seen
  ARRIVED at the BLUE cone: lidar range 0.284 m -> estimated gap 2.6 cm (1.0 in)
```

You can type the full name, a unique prefix (`b`) or the number (`3`). Type `q`
to quit. **Ctrl-C stops the robot at any time.**

Images to show the instructor, in `hw9_output/`:

- `task1_<colour>.png`: the camera frame where that cone was most centred,
  with every detection boxed and labelled with its colour name
- `task1_map.png`: top-down view with the robot, the cones and the lidar scan
  (0.5 m grid)

## Parameters

Set them in `config/hw9_params.yaml`. Editing it needs no rebuild. You can also
set them on the command line with `-p name:=value`. Write decimals with a point
(`0.30`).

| Parameter | Default | Meaning |
|---|---|---|
| `colors` | `[orange, green, blue]` | The three cone colour names |
| `hsv.<colour>` | see file | HSV box(es), OpenCV units: `[h_lo, h_hi, s_lo, s_hi, v_lo, v_hi]` |
| `contact_range` | `0.28` | Lidar range when the bumper touches a cone. **Calibrate** (Step 1c) |
| `target_gap` | `0.025` | Gap to leave (m) |
| `image_topic` | `/oakd/rgb/preview/image_raw` | Camera topic. A topic ending in `compressed` is decoded as JPEG |
| `camera_hfov_deg` | `42.0` | Width of the camera view, in degrees. The preview is a centre crop of the 69° image. Use 69 if a round object looks squashed |
| `scan_speed` | `0.30` | Task 1 turn rate (rad/s). It is lowered automatically to keep at least one image per 2.5° |
| `max_speed` / `min_speed` | `0.15` / `0.025` | Task 2 speed far from / close to the cone (m/s) |
| `backoff_distance` | `0.08` | Reverse distance before turning to the next cone (m). 0 turns it off |
| `max_cone_range` | `2.5` | Objects further away are not treated as cones (m) |
| `laser_yaw_offset` | `π/2` | Lidar yaw on the robot (TB4: +90°, see hw6) |
| `laser_x_offset` | `-0.04` | Lidar position ahead of `base_link` (m). Only used for position estimates |
| `output_dir` | `hw9_output` | Where images are written |
| `cmd_vel_stamped` | `true` | `TwistStamped` on `/cmd_vel` (Jazzy). Set `false` for plain `Twist` |

## Troubleshooting

- **`Waiting for camera ... / lidar ...`, then an error.** The robot is docked
  (Step 0), or the terminal is missing the robot profile.
- **No `/oakd/rgb/preview/image_raw`.** Run `ros2 topic list | grep -i oakd` and
  pass the topic you find with `-p image_topic:=...`.
- **`The robot is not turning`.** Odometry didn't change while turning was
  commanded. The robot may still be docked, `/cmd_vel` may not be reaching it,
  or it may be one of the misbehaving robots listed in
  `robot_profiles/readme.md`.
- **A cone is NOT FOUND, or it locks onto the wrong object.** Redo Step 1a and
  look at `task1_<colour>.png` and the masks in `calib_latest.png`.
- **`X and Y were detected on the same cone`.** The two HSV boxes overlap.
  Narrow them.
- **`camera only, the lidar did not see it` / `steering by camera`.** The lidar
  does not see that cone: it is too short, or further than `max_cone_range`.
  The robot then stops when the cone fills the camera view, well short of an
  inch, and says the gap is unknown. Use cones taller than about 25 cm.
- **It stops too far or too close.** The printed `estimated gap` is
  `lidar range - contact_range`. If the real gap differs, adjust
  `contact_range` by the difference: larger stops further away. Re-check
  Step 1c.
- **`Create 3 backup limit` warning.** The Create 3 refused to reverse. Set
  `backoff_distance: 0.0`; turning in place next to a cone is fine because the
  robot is round.
- **`time stamps do not match /odom`.** The robot's clocks are out of sync. It
  still works, with slightly less accurate Task 1 directions.
