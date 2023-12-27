
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

```shell
rosdep install --from-paths ./ --ignore-src -r -y
```

> .clang-format 文件搭配 clang-format 插件进行代码格式化
