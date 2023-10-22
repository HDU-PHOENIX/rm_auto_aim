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

#include "ukf_plus.h"
#include "auto_aim_interfaces/msg/rune.hpp"
#include "ukf_plus.h"
// #include "auto_aim_interfaces/msg/target.hpp"

namespace rune
{
// 装甲追踪器类
class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  // using Armors = auto_aim_interfaces::msg::Armors;
  // using Armor = auto_aim_interfaces::msg::Armor;
  using Rune = auto_aim_interfaces::msg::Rune;


  // 初始化追踪器
  // void init(const Armors::SharedPtr & armors_msg);

  // 更新追踪器
  // void update(const Rune::SharedPtr & rune_msg);


  double info_position_diff; // 位置差
  double info_yaw_diff; // 偏航角差

  Eigen::VectorXd measurement; // 测量值

  Eigen::VectorXd target_state; // 目标状态

  Filter* ukf; // ukf滤波器
  // 用于存储另一对装甲消息
  // double dz, another_r;

private:
  // 初始化扩展卡尔曼滤波器
  // void initEKF(const Armor & a);

  // 更新装甲数量
  // void updateArmorsNum(const Armor & a);

  // 处理装甲跳变
  // void handleArmorJump(const Armor & a);

  // 将方向转换为偏航角
  // double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  // 从状态中获取装甲位置
  // Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);



  // double max_match_distance_; // 最大匹配距离
  // double max_match_yaw_diff_; // 最大匹配偏航角差

  // int detect_count_; // 检测计数
  // int lost_count_; // 丢失计数

  // double last_yaw_; // 上一次的偏航角
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
