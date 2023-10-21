
```
.
├── armor_auto_aim
│   ├── armor_detector
│   └── armor_tracker
|
├── rune_auto_aim
│   ├── rune_detector
│   └── rune_tracker
|
├── sensor
│   └── camera
|
|── auto_aim_interfaces
|
└── auto_aim

rosdep install --from-paths ./ --ignore-src -r -y
```

## armor_auto_aim

装甲板自瞄部分，包含 `armor_detector` `armor_tracker` 包

### armor_detector

装甲板识别 `armor_detector_node` 节点

### armor_tracker

装甲板追踪

## rune_auto_aim

符自瞄部分 ...

## sensor

### camera

相机包 `camera_node` 节点

## auto_aim_interfaces

自瞄自定义接口，定义消息类型

## auto_aim

接收下位机消息和图片消息，判断 `armor_auto_aim` or `rune_auto_aim`

> cmake 中使用 clang 作为编译器
>
> .clang-format 文件搭配 clang-format 插件进行代码格式化
