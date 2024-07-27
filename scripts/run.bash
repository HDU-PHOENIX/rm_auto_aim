#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

cd "$(dirname "$0")"
cd ..

sudo chmod 777 /dev/ttyACM0

ros2 launch auto_aim launch.py &
pid2=$!
echo -e "auto_aim pid: $pid2\n\n"

bash ./communicate/reset.bash &
pid1=$!
echo -e "communicate_node pid: $pid1\n\n"


# 当捕获到 SIGINT 信号时，结束这两个进程
trap "kill $pid2 $pid1" SIGINT

# 等待所有后台进程结束
wait
