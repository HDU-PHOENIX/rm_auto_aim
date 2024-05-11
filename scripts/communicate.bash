#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

export ROS_LOG_DIR=/home/rm/rm_auto_aim/log

lsusb | sed 's/:/ /g' | awk '{print $2, $4}' | sed 's/ /\//g' | xargs -I {} sudo usbreset /dev/bus/usb/{}
sleep 1
sudo chmod 777 /dev/ttyACM0
ros2 launch communicate launch.py