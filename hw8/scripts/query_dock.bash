#!/bin/bash
ros2 topic echo /dock_status --once |grep is_docked

