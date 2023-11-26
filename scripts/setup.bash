#!bash

sudo apt update
echo ">>> start install wget"
sudo apt install wget -y
echo ">>> successfully install wget\n"

echo "\n>>> start install openvino\n"
sleep 1
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB -O /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
if [ -f /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB ]; then
    echo "successfully wget GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB"
else
    echo "wget GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB failed"
    exit -1
fi

sudo apt-key add /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB 
echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list # ubuntu22
sudo apt update && sudo apt install openvino -y
echo ">>> successfully install openvino"

echo "\n>>> start install ceres\n"
sleep 1
sudo apt install libceres-dev -y
echo ">>> successfully install ceres"

echo "\n>>> start install ros2\n"
sleep 1
if ! command -v ros2 &> /dev/null
then
    echo "ROS is not installed, start install ROS2"
    wget http://fishros.com/install -O fishros && . fishros
else
    echo "ROS installed, skip install ROS2"
fi

echo "\n>>> start install ros2 dependencies\n"
sleep 1
if [ -f /etc/ros/rosdep/sources.list.d ]; then
    echo "rosdep already init"
else
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths ./ --ignore-src -r -y
