#!/usr/bin/env bash
source ~/.bashrc

# angle speed ω = forward speed v / circle radius R
# diameter is 50 cm
#ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped \
#"twist:
#  linear:
#    x: 0.15
#    y: 0.0
#    z: 0.0
#  angular:
#    x: 0.0
#    y: 0.0
#    z: 0.60"

# diameter is 75 cm
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped \
"twist:
  linear:
    x: 0.15
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.40"
