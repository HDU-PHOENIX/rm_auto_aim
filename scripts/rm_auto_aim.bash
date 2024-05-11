#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

export ROS_LOG_DIR=/home/rm/rm_auto_aim/log

ros2 launch auto_aim launch.py