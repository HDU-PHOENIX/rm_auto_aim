#include "rune_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rune {
Tracker::Tracker() {
    ukf = new UKF_PLUS(0, true, false, 1.5, 1.2); //ukf滤波器初始化 原1.5  1.2
}

// void Tracker::update(const Rune::SharedPtr & rune_msg)
// {
//   // UKF 预测

//   RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"),"UKF predict");

// }

// void Tracker::initEKF(const Armor & a)
// {
//   double xa = a.pose.position.x;
//   double ya = a.pose.position.y;
//   double za = a.pose.position.z;
//   last_yaw_ = 0;
//   double yaw = orientationToYaw(a.pose.orientation);

//   // 将初始位置设置为目标后方 0.2 米处
//   target_state = Eigen::VectorXd::Zero(9);
//   double r = 0.26;
//   double xc = xa + r * cos(yaw);
//   double yc = ya + r * sin(yaw);
//   dz = 0, another_r = r;
//   target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

//   ekf.setState(target_state);
// }

// void Tracker::updateArmorsNum(const Armor & armor)
// {
//   if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
//     tracked_armors_num = ArmorsNum::BALANCE_2;
//   } else if (tracked_id == "outpost") {
//     tracked_armors_num = ArmorsNum::OUTPOST_3;
//   } else {
//     tracked_armors_num = ArmorsNum::NORMAL_4;
//   }
// }

// void Tracker::handleArmorJump(const Armor & current_armor)
// {
//   double yaw = orientationToYaw(current_armor.pose.orientation);
//   target_state(6) = yaw;
//   updateArmorsNum(current_armor);
//   // 只有 4 个装甲具有 2 个半径和高度
//   if (tracked_armors_num == ArmorsNum::NORMAL_4) {
//     dz = target_state(4) - current_armor.pose.position.z;
//     target_state(4) = current_armor.pose.position.z;
//     std::swap(target_state(8), another_r);
//   }
//   RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

//   // 如果位置差异大于 max_match_distance_，将此情况视为 EKF 发散，重置状态
//   auto p = current_armor.pose.position;
//   Eigen::Vector3d current_p(p.x, p.y, p.z);
//   Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
//   if ((current_p - infer_p).norm() > max_match_distance_) {
//     double r = target_state(8);
//     target_state(0) = p.x + r * cos(yaw);  // xc
//     target_state(1) = 0;                   // vxc
//     target_state(2) = p.y + r * sin(yaw);  // yc
//     target_state(3) = 0;                   // vyc
//     target_state(4) = p.z;                 // za
//     target_state(5) = 0;                   // vza
//     RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"),  "Reset State!");
//   }

//   ekf.setState(target_state);
// }

// double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
// {
//   // 获取装甲偏航角
//   tf2::Quaternion tf_q;
//   tf2::fromMsg(q, tf_q);
//   double roll, pitch, yaw;
//   tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
//   // 使偏航角连续变化（-pi~pi 转换为 -inf~inf）
//   yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
//   last_yaw_ = yaw;
//   return yaw;
// }

// Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
// {
//   // 计算当前装甲的预测位置
//   double xc = x(0), yc = x(2), za = x(4);
//   double yaw = x(6), r = x(8);
//   double xa = xc - r * cos(yaw);
//   double ya = yc - r * sin(yaw);
//   return Eigen::Vector3d(xa, ya, za);
// }

} // namespace rune
