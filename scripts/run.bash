source /opt/ros/humble/setup.bash
source ./install/setup.bash

sudo chmod 777 /dev/ttyACM0
ros2 launch auto_aim launch.py
