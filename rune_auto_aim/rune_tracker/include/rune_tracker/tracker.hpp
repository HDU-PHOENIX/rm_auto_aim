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
// װ��׷������
class Tracker {
public:
    Tracker();
    using Rune = auto_aim_interfaces::msg::Rune;

    double info_position_diff; // λ�ò�
    double info_yaw_diff;      // ƫ���ǲ�

    Eigen::VectorXd measurement; // ����ֵ

    Eigen::VectorXd target_state; // Ŀ��״̬

    Filter* ukf; // ukf�˲���

private:
};

} // namespace rune

#endif // ARMOR_PROCESSOR__TRACKER_HPP_
