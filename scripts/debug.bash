#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
pid0=$!  # 保存进程的 PID
echo -e "foxglove_bridge pid: $pid0\n\n"
sleep 2

# 当捕获到 SIGINT 信号时，结束这两个进程
trap "kill $pid0" SIGINT

# 等待所有后台进程结束
wait
