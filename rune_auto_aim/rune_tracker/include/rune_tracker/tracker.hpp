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

#include "rune_tracker/ukf_plus.h"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "ukf_plus.h"
// #include "auto_aim_interfaces/msg/target.hpp"

namespace rune
{
// װ��׷������
class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  // ��ʼ��׷����
  void init(const Armors::SharedPtr & armors_msg);

  // ����׷����
  void update(const Armors::SharedPtr & armors_msg);

  UKF_PLUS ukf; // ��չ�������˲���

  int tracking_thres; // ׷����ֵ
  int lost_thres; // ��ʧ��ֵ

  // ׷����״̬ö��
  enum State {
    LOST, // ��ʧ״̬
    DETECTING, // ���״̬
    TRACKING, // ׷��״̬
    TEMP_LOST, // ��ʱ��ʧ״̬
  } tracker_state;

  std::string tracked_id; // ׷�ٵ�ID
  Armor tracked_armor; // ׷�ٵ�װ��
  ArmorsNum tracked_armors_num; // ׷�ٵ�װ������

  double info_position_diff; // λ�ò�
  double info_yaw_diff; // ƫ���ǲ�

  Eigen::VectorXd measurement; // ����ֵ

  Eigen::VectorXd target_state; // Ŀ��״̬

  // ���ڴ洢��һ��װ����Ϣ
  double dz, another_r;

private:
  // ��ʼ����չ�������˲���
  void initEKF(const Armor & a);

  // ����װ������
  void updateArmorsNum(const Armor & a);

  // ����װ������
  void handleArmorJump(const Armor & a);

  // ������ת��Ϊƫ����
  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  // ��״̬�л�ȡװ��λ��
  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_; // ���ƥ�����
  double max_match_yaw_diff_; // ���ƥ��ƫ���ǲ�

  int detect_count_; // ������
  int lost_count_; // ��ʧ����

  double last_yaw_; // ��һ�ε�ƫ����
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
