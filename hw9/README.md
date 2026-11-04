# Homework 9 — Find coloured traffic cones and drive to them

The `hw9pkg` package uses the OAK-D colour camera to tell **which** cone is
where, and the lidar to measure **exactly how far** away it is. The depth
camera is not needed: it is off by default on the TurtleBot 4 and cannot
measure closer than about 0.3 m, which is exactly where the last inch matters.

| Program | What it does |
|---|---|
| `cone_finder` | **Task 1:** turns 360° in 24° steps, finds each colour in the camera image, matches it to a lidar object, and prints every cone's colour, bearing and distance. **Task 2:** loop that asks for a colour and drives to about 2.5 cm (1 inch) from that cone |
| `calibrate` | Checks and tuning before the demo, without moving the robot: HSV sliders per colour, camera snapshots, camera + lidar check, and the lidar range at which a cone touches the bumper |

How Task 2 gets within an inch: it turns toward the stored cone position, finds
the cone again with the camera, then drives in **measure → drive** steps. The
robot stops 0.25 m short, measures with the lidar while standing still, drives the
rest with odometry, measures again, and corrects until the gap is 2.5 cm ± 8 mm.
Within 0.55 m the cone fills the camera view, so from there it tracks the nearest
small lidar object straight ahead. While driving forward it stops immediately if
anything comes within 8 mm of touching.

## Setup (every terminal)

On the lab laptop, from the repo root (for example
`~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems`). Robot F worked for hw7 and hw8:

```bash
cd ~/Desktop/cs5335/CS5335-Robotic-Science-and-Systems
git pull
source robot_profiles/select_robot.bash f
```

Do **not** run `simmode` (hw8) in these terminals: that is for simulation only.

## Build (once, and after any code change)

```bash
cd hw9
colcon build --packages-select hw9pkg
source install/setup.bash
```

Run everything below from the `hw9` folder, so `hsv_ranges.yaml` and
`hw9_output/` land there. If the build or a run complains about `cv2` or `yaml`:

```bash
sudo apt install python3-opencv python3-yaml
```

## 1. Check the topics

```bash
ros2 topic hz /oakd/rgb/preview/image_raw
ros2 topic hz /scan
ros2 topic hz /odom
```

All three must show a rate. If the camera topic is missing, look for the camera topics that are there:

```bash
ros2 topic list | grep oakd
```

Then pass one of them to every command with
`--ros-args -p image_topic:=<topic>`. A topic ending in `/compressed` works too, and
uses less Wi-Fi.

## 2. Tune the three colours (lab lighting changes them)

Put one cone about 1 m in front of the robot, in the middle of the view:

```bash
ros2 run hw9pkg calibrate snap
```

This saves `hw9_output/snap_01.png` (+ `_detected.png`) and prints the H, S, V at the
image centre and which colours it detects. Then tune each colour with live sliders:

```bash
ros2 run hw9pkg calibrate tune orange
```

Move the sliders until **only the cone** is white in the right-hand mask. Clicking the
picture prints the HSV at that pixel. Press `s` to save to `hsv_ranges.yaml`, `q` to quit.
Repeat for the other two colours. Notes:

- OpenCV hue runs 0–179. Starting values are orange 7–20, yellow 21–35, green
  40–85, blue 95–130, purple 131–165, and red 170→6 (wraps around).
- Raise **S lo** to reject the grey floor and white walls. Raise **V lo** to reject dark shadows.
- With all three cones in view, `calibrate snap` should list each colour once, and
  no colour should light up on the wrong cone (red and orange are the usual mix-up).

Tuning on a saved picture also works without the robot:

```bash
ros2 run hw9pkg calibrate tune orange --image hw9_output/snap_01.png
```

## 3. Measure `touch_range` (important for the 1-inch stop)

Place a cone **touching** the front bumper, straight ahead:

```bash
ros2 run hw9pkg calibrate lidar
```

It prints something like
`If the cone is touching the front bumper, run cone_finder with: -p touch_range:=0.205`.
Use that number below. It includes the cone's wide base, which the bumper hits before
the lidar's height. The robot stops at `touch_range + 0.025 m`.

If it prints `Nothing within 1.5 m straight ahead`, the cone may be shorter than the
lidar's height. Check `ros2 topic echo /scan --once` with the cone in front; if the
lidar never sees the cones, this approach needs the depth camera instead.

## 4. Check camera + lidar together, without moving

Set up the triangle: 3 cones about 1.5 m or more apart, nothing inside, robot in the
middle. Turn the robot by hand so one or two cones are in view:

```bash
ros2 run hw9pkg calibrate look
```

For every visible cone it prints the camera angle and the lidar angle, distance and
width. The two angles should agree within a few degrees (+ = left).

**Camera angle check:** if the camera angle is off by more than ~5° for every cone, the
camera_info does not match the preview image. Run every command with
`-p use_camera_info:=false -p hfov_deg:=69.0`. If the camera angles are then too small
(nearer 0 than the lidar's), increase `hfov_deg`; if they are too large, decrease it. The
lidar refines the angle anyway, so a few degrees off is fine.

## 5. Run the assignment (demo)

Put the robot back in the centre of the triangle:

```bash
ros2 run hw9pkg cone_finder --ros-args -p colors:=orange,green,blue -p touch_range:=0.205
```

Replace the three colours with your cones' colours and use your measured `touch_range`.

**Task 1** runs immediately: 15 looks, 24° apart, then it is back at its starting
heading. It prints each look, and then a summary like:

```
Cones found (bearing from the starting heading, + = left / counter-clockwise):
  ORANGE   bearing   +47.0 deg   distance 0.82 m
  GREEN    bearing  -71.0 deg   distance 0.91 m
  BLUE     bearing +168.0 deg   distance 0.86 m
```

The summary is also saved to `hw9_output/cones.txt`, and the camera frame from each look
(with the cones boxed) to `hw9_output/sweep_01.png` … `sweep_15.png`.

**Task 2** then asks in a loop:

```
Which cone? (orange/green/blue, list, quit): green
```

Type a colour and the robot drives there. It prints each step and finally
`Arrived at the GREEN cone: bumper gap about 2.4 cm (target 2.5 cm)`. Before the next
cone it backs up 0.30 m first. `list` reprints the Task 1 summary. `quit` or Ctrl-C ends
the program; the robot is told to stop in every case.

Keep someone next to the robot during the demo.

## Parameters

Add any of these after `--ros-args` as `-p name:=value`.

| Parameter | Default | Meaning |
|---|---|---|
| `colors` | `orange,green,blue` | The three cone colours (hard-coded names, see `hsv_ranges.yaml`) |
| `touch_range` | `0.20` | Lidar range (m) when a cone touches the bumper — **measure it** (step 3) |
| `stop_gap` | `0.025` | Gap (m) to leave between bumper and cone |
| `arrive_tolerance` | `0.008` | Accept the stop when within this (m) of the target gap |
| `final_stage` | `0.25` | Stop this far (m) short, re-measure, then drive the rest |
| `backoff_distance` | `0.30` | Back away this far (m) from a reached cone before the next one |
| `sweep_step_deg` | `24.0` | Angle between looks in Task 1; keep it below the camera's field of view |
| `max_cone_range` | `3.0` | Ignore lidar objects further than this (m) |
| `max_cone_width` | `0.45` | Lidar objects wider than this (m) are walls, not cones |
| `lidar_window_deg` | `12.0` | Search the lidar ± this around the camera's angle |
| `camera_handoff_range` | `0.55` | Closer than this (m), track the cone with the lidar only |
| `max_linear_speed` / `min_linear_speed` | `0.12` / `0.03` | m/s |
| `max_angular_speed` / `min_angular_speed` | `0.6` / `0.12` | rad/s |
| `min_valid_range` | `0.12` | Lidar rays shorter than this hit the robot itself and are ignored |
| `laser_yaw_offset` | `π/2` | Lidar yaw relative to `base_link` (rad), as in hw6 / hw7 |
| `image_topic` | `/oakd/rgb/preview/image_raw` | Camera topic (`.../compressed` also works) |
| `camera_info_topic` | `/oakd/rgb/preview/camera_info` | Used for the pixel → angle conversion |
| `use_camera_info` / `hfov_deg` | `true` / `69.0` | Fall back to this horizontal field of view without camera_info |
| `hsv_file` | `hsv_ranges.yaml` | Colour ranges written by `calibrate tune` |
| `output_dir` | `./hw9_output` | Sweep images and `cones.txt` |

## Troubleshooting

- **A colour is "NOT FOUND" in Task 1:** look at its `sweep_XX.png` images. If the cone is
  not boxed, re-tune that colour (step 2). If the sweep says
  `seen by the camera ... but the lidar found no object there`, check step 3 (cone height)
  and `calibrate look`. Asking for a missing colour in Task 2 sweeps again for it.
- **The wrong thing is detected** (a poster, a chair): raise S lo / V lo for that colour,
  or move the object. Anything wider than 0.45 m on the lidar is ignored already.
- **It stops short or too close:** re-measure `touch_range` with the cone touching the
  bumper. A bigger `touch_range` stops further away.
- **`Safety stop: something ... ahead`:** something was about to touch the bumper. Check
  `touch_range` and `safety_margin` (default 0.008 m).
- **`turn ... timed out` / `odometry stopped publishing`:** the robot is not following
  `/cmd_vel` (robot B's known issue in `robot_profiles/readme.md`). Check
  `ros2 topic hz /odom`, or use another robot.
- **`calibrate lidar` reports an object straight ahead with nothing in front of the robot:**
  the lidar is hitting the robot's own standoffs. Raise `min_valid_range` just past that
  distance (but keep it well below `touch_range`).
