#!/usr/bin/env bash
ros2 service call /move_turn/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
