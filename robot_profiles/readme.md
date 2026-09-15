# TurtleBot Connection Profiles and Known Issues

This folder stores ROS 2 connection profiles for the TurtleBots used in CS5335.

## Before controlling any robot

1. Make sure the robot has open space around it and a person is watching it.
2. Switch to the correct profile:
   ```bash
   source ~/Documents/cs5335/robot_profiles/select_robot.bash b
   ```
   Replace b with c or f as needed.
3. Confirm that the ROS connection is live:
   ```bash
   ros2 topic list --no-daemon --spin-time 5
   ```
4. Do not drive the robot if /cmd_vel, /scan, and /odom do not appear.
5. Do not change E-stop settings or robot system configuration.


## Robot B: velocity-control issue
Known connection values:
- IP: 10.245.151.196
- ROS Domain ID: 0
- Discovery Server ID: 6

Switch to it:
```bash
source ~/Documents/cs5335/robot_profiles/select_robot.bash b
```
### Observed behavior
- ROS topics were visible from the Ubuntu VM.
- Keyboard teleoperation and /cmd_vel messages could be published.
- The TurtleBot sometimes did not move even when the command was visible on /cmd_vel.
- A /drive_distance action run from the TurtleBot SSH terminal succeeded once, showing that the robot could physically move.
- On a later attempt, /drive_distance accepted a goal but did not return a result. This behavior was intermittent.

Use an open area. Run keyboard teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```
Press i to command forward motion.
Expected behavior: move straight forward.
Observed issue: messages may be published, but the robot may not move.


## Robot C: unexpected right-turn behavior
Known connection values:
- IP: 10.245.151.147
- ROS Domain ID: 5
- Discovery Server ID: 0

Switch to it:
```bash
source ~/Documents/cs5335/robot_profiles/select_robot.bash c
```

### Observed behavior
- A straight command was sent with linear.x > 0 and angular.z = 0.
- /cmd_vel confirmed that angular.z was zero.
- Despite this, the robot turned right instead of traveling straight.
- Keyboard teleoperation with i also showed an abnormal pattern: move forward briefly, turn right about 90 degrees, move again, then turn again.

Use an open area and run:
```bash
timeout 2s ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped \
"{twist: {linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```
Expected behavior: travel straight forward.
Observed issue: robot may turn right even though the command specifies zero angular velocity.