# RM_AUTO_AIM 2024

- [RM\_AUTO\_AIM 2024](#rm_auto_aim-2024)
  - [Debug](#debug)
    - [Prepare](#prepare)
    - [Compile](#compile)
    - [Command](#command)
  - [Readme 传送门](#readme传送门)

## Debug

### Prepare

- 安装依赖

```shell
bash ./scripts/setup.bash
# rosdep install --from-paths ./ --ignore-src -r -y
```

- 启用通信子模块

```shell
git submodule init
git submodule update # git pull 之后使用需要使用此命令同步子模块更新
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

```shell
bash ./scripts/run.bash
bash ./scripts/debug.bash
```

## Readme 传送门

- [能量机关](./rune_auto_aim/README.md)
- [装甲板](./armor_auto_aim/README.md)
