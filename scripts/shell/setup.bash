#!bash

cd $(dirname ${BASH_SOURCE[0]} )

bash ./openvino.bash # openvino
sudo apt install libceres-dev -y

if ! command -v ros2 &> /dev/null
then
    echo "ROS 未安装，正在执行安装命令..."
    curl http://fishros.com/install -o fishros && . fishros
else
    echo "ROS 已安装"
fi

sudo rosdep init
rosdep update
rosdep install --from-paths ./ --ignore-src -r -y
