#!/usr/bin/env bash
source ~/.bashrc
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
