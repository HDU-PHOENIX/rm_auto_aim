# RM_AUTO_AIM 2024
- [RM\_AUTO\_AIM 2024](#rm_auto_aim-2024)
	- [Brief](#brief)
	- [Debug](#debug)
		- [Prepare](#prepare)
		- [Compile](#compile)
		- [Command](#command)
	- [Readme传送门](#readme传送门)

## Brief
```
.
├──armor_auto_aim
│  ├──armor_detector
│  ├──armor_shooter
│  └──armor_tracker
├──auto_aim
│  ├──config
│  └──launch
├──auto_aim_interfaces
│  └──msg
├──rune_auto_aim
│  ├──rune_detector
│  ├──rune_shooter
│  └──rune_tracker
├──scripts
├──sensor
│  ├──camera
│  ├──camerainfo
│  └──serial
└──tools
```

## Debug
### Prepare
- 安装依赖
	```shell
	rosdep install --from-paths ./ --ignore-src -r -y
	```
- 启用通信子模块
	```shell
	git submodule init
	git submodule update
	```
- .clang-format 文件搭配 clang-format 插件进行代码格式化


### Compile
```shell
colcon build
source install/setup.sh
```

### Command
```shell
ros2 launch auto_aim launch.py
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Readme传送门
- [能量机关](./rune_auto_aim/README.md)
- [装甲板](./armor_auto_aim/armor_tracker/README.md)