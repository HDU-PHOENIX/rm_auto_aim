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
// װ��׷������
class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  // using Armors = auto_aim_interfaces::msg::Armors;
  // using Armor = auto_aim_interfaces::msg::Armor;
  using Rune = auto_aim_interfaces::msg::Rune;


  // ��ʼ��׷����
  // void init(const Armors::SharedPtr & armors_msg);

  // ����׷����
  // void update(const Rune::SharedPtr & rune_msg);


  double info_position_diff; // λ�ò�
  double info_yaw_diff; // ƫ���ǲ�

  Eigen::VectorXd measurement; // ����ֵ

  Eigen::VectorXd target_state; // Ŀ��״̬

  Filter* ukf; // ukf�˲���
  // ���ڴ洢��һ��װ����Ϣ
  // double dz, another_r;

private:
  // ��ʼ����չ�������˲���
  // void initEKF(const Armor & a);

  // ����װ������
  // void updateArmorsNum(const Armor & a);

  // ����װ������
  // void handleArmorJump(const Armor & a);

  // ������ת��Ϊƫ����
  // double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  // ��״̬�л�ȡװ��λ��
  // Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);



  // double max_match_distance_; // ���ƥ�����
  // double max_match_yaw_diff_; // ���ƥ��ƫ���ǲ�

  // int detect_count_; // ������
  // int lost_count_; // ��ʧ����

  // double last_yaw_; // ��һ�ε�ƫ����
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
