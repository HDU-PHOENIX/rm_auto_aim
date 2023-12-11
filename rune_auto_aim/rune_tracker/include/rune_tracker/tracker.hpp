// Copyright 2023 wangchi

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/rune.hpp"
#include "ukf_plus.h"

namespace rune {
// 装甲板追踪器类
class Tracker {
public:
    Tracker();
    using Rune = auto_aim_interfaces::msg::Rune;

    double info_position_diff; // 位置差
    double info_yaw_diff;      // 偏航角差

    Eigen::VectorXd measurement; // 测量值

    Eigen::VectorXd target_state; // 目标状态

    Filter* ukf; // ukf滤波器

private:
};

} // namespace rune

#endif // ARMOR_PROCESSOR__TRACKER_HPP_
