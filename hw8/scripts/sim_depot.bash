#!/bin/bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=depot map:=/opt/ros/jazzy/share/turtlebot4_navigation/maps/depot.yaml