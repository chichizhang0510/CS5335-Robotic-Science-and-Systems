#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash

case "${1:-}" in
  b)
    robot_name="B"
    domain_id=0
    server_id=6
    robot_ip="10.245.151.196"
    ;;
  c)
    robot_name="C"
    domain_id=5
    server_id=0
    robot_ip="10.245.151.147"
    ;;
  f)
    robot_name="F"
    domain_id=0
    server_id=33
    robot_ip="10.245.131.97"
    ;;
  *)
    echo "Usage: source select_robot.bash {b|c|f}"
    return 2
    ;;
esac

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=True
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE

export ROS_DOMAIN_ID="$domain_id"
export ROS_DISCOVERY_SERVER="$(printf '%*s' "$server_id" '' | tr ' ' ';')${robot_ip}:11811;"

ros2 daemon stop >/dev/null 2>&1 || true

echo "Switched to TurtleBot ${robot_name}: ${robot_ip}, Domain ${domain_id}, Server ID ${server_id}"