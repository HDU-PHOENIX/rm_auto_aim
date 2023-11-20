#!bash

bash ./openvino.bash # openvino
sudo apt install libceres-dev -y
rosdep install --from-paths ./ --ignore-src -r -y
