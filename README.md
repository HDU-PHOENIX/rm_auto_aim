# RM_AUTO_AIM 2024

- [RM\_AUTO\_AIM 2024](#rm_auto_aim-2024)
  - [Debug](#debug)
    - [Prepare](#prepare)
    - [Compile](#compile)
    - [Command](#command)
  - [Readme 传送门](#readme传送门)

## Debug

### Prepare

```shell
mkdir -p rm_auto_aim_ws/src && cd rm_auto_aim_ws
git clone https://github.com/HDU-PHOENIX/rm_auto_aim.git src
```

```shell
bash ./src/scripts/setup.bash
```

`.clang-format` 文件搭配 clang-format 插件进行代码格式化

`.clang-tidy` 文件使用 clangd 插件默认开启，使用 C/C++ 插件需要在 C_Cpp.codeAnalysis.clangTidy.enabled 中开启

### Compile

```shell
colcon build --symlink-install
source ./install/setup.bash
```

### Run

```shell
bash ./scripts/run.bash
bash ./scripts/debug.bash
```

## Readme 传送门

- [能量机关](./rune_auto_aim/README.md)
- [装甲板](./armor_auto_aim/README.md)
