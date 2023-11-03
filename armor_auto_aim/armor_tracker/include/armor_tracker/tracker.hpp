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

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim {

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

// 装甲追踪器类
class Tracker {
public:
    Tracker(double max_match_distance, double max_match_yaw_diff);

    using Armors = auto_aim_interfaces::msg::Armors;
    using Armor = auto_aim_interfaces::msg::Armor;

    // 初始化追踪器
    void Init(const Armors::SharedPtr& armors_msg);

    // 更新追踪器
    void Update(const Armors::SharedPtr& armors_msg);

    ExtendedKalmanFilter ekf; // 扩展卡尔曼滤波器

    int tracking_thres; // 追踪阈值
    int lost_thres;     // 丢失阈值

    // 追踪器状态枚举
    enum State {
        LOST,      // 丢失状态
        DETECTING, // 检测状态
        TRACKING,  // 追踪状态
        TEMP_LOST, // 暂时丢失状态
    } tracker_state;

    std::string tracked_id;       // 追踪的ID
    Armor tracked_armor;          // 追踪的装甲
    ArmorsNum tracked_armors_num; // 追踪的装甲数量

    double info_position_diff; // 位置差
    double info_yaw_diff;      // 偏航角差

    Eigen::VectorXd measurement; // 测量值

    Eigen::VectorXd target_state; // 目标状态

    // 用于存储另一对装甲消息
    double dz, another_r;

private:
    // 初始化扩展卡尔曼滤波器
    void InitEKF(const Armor& a);

    // 更新装甲数量
    void UpdateArmorsNum(const Armor& a);

    // 处理装甲跳变
    void HandleArmorJump(const Armor& a);

    // 将方向转换为偏航角
    double OrientationToYaw(const geometry_msgs::msg::Quaternion& q);

    // 从状态中获取装甲位置
    Eigen::Vector3d GetArmorPositionFromState(const Eigen::VectorXd& x);

    double max_match_distance_; // 最大匹配距离
    double max_match_yaw_diff_; // 最大匹配偏航角差

    int detect_count_; // 检测计数
    int lost_count_;   // 丢失计数

    double last_yaw_; // 上一次的偏航角
};

} // namespace rm_auto_aim

#endif // ARMOR_PROCESSOR__TRACKER_HPP_
