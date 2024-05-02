#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

sudo chmod 777 /dev/ttyACM0

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
pid0=$!  # 保存进程的 PID
echo -e "foxglove_bridge pid: $pid0\n\n"
sleep 2

# ros2 run communicate communicate_node &
# pid1=$!
# echo -e "communicate_node pid: $pid1\n\n"
# sleep 5

ros2 launch auto_aim launch.py &
pid2=$!
echo -e "auto_aim pid: $pid2\n\n"


# 当捕获到 SIGINT 信号时，结束这两个进程
trap "kill $pid0 $pid2" SIGINT

# 等待所有后台进程结束
wait
