# Homework 5 — Odometry accuracy

`odom_lab.py` handles the bookkeeping for each trial. It:

- captures `/odom` before and after the run
- prints the motion that odometry reports
- asks for your ruler measurement
- appends one row to a CSV file

`odom_lab.py summary` prints the measurement table with the mean and
standard deviation of the error, ready to show the instructor.

It is a plain `rclpy` script, so it needs no `colcon build`.

## Setup (every terminal)

```bash
cd ~/CS5335-Robotic-Science-and-Systems
source robot_profiles/select_robot.bash f
```

## Task 1 — look at an Odometry message

```bash
ros2 topic echo /odom            # 20 Hz stream, Ctrl-C to stop
ros2 topic echo /odom --once     # a single message
mkdir -p hw5/data && ros2 topic echo /odom --once > hw5/data/odom_sample.yaml
```

## Task 2 — 10 × 1 m with the hw4 driving action

Terminal 1, the hw4 nodes:

```bash
source hw4/install/setup.bash
ros2 launch day4pkg day4.launch.py
```

Terminal 2, the recorder:

```bash
cd hw5
python3 odom_lab.py record --method my_action --mode line --target 1.0
```

For each trial:

1. Mark the start and press Enter. The script captures the start odometry.
2. Type `move 1.0` in terminal 1.
3. Once the robot stops, press Enter. The script captures the end odometry.
4. Type the ruler distance in metres.

## Task 3 — circle of radius 0.5 m

```bash
python3 odom_lab.py record --method my_action --mode circle --trials 1
```

Drive the circle with `arc left 0.5 360` in terminal 1. Then measure how far
the robot ended up from the start mark. The target is 0 m, so the error is
the distance itself.

## Task 4 — 10 × 1 m with the built-in DriveDistance action

Here the recorder sends the `/drive_distance` goal itself, so terminal 1 is
not needed:

```bash
python3 odom_lab.py record --method drive_distance --mode line --drive-distance
```

`--speed` sets `max_translation_speed`. It defaults to 0.10 m/s, the same
speed the hw4 driving node uses, so the two methods are compared fairly. To
send the goal by hand instead:

```bash
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance \
  "{distance: 1.0, max_translation_speed: 0.10}"
```

## Results

```bash
python3 odom_lab.py summary          # reads data/trials.csv
```

This prints one table per method and mode. In each table:

- **Actual error** = ruler measurement − target
- **Odometry error** = straight-line distance between the start and end
  `/odom` positions − target
- **Std dev** is the sample standard deviation (n − 1)

Each row also includes the heading change odometry reported. Every captured
`/odom` message is saved in full under `data/trials_odom/`. The CSV keeps
the forward and sideways parts of each motion as well.

Task 4 compares the actual-error rows of `my_action` against `drive_distance`.
To compare accuracy, look at both the mean (bias) and the standard
deviation (repeatability).

Type `q` at any prompt to stop early. Re-running `record` with the same
`--method` and `--mode` continues the trial numbering.
