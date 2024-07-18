#include "armor_tracker/tracker.hpp"

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

namespace armor {
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff):
    tracker_state(LOST),
    tracked_id(std::string("")),
    measurement(Eigen::VectorXd::Zero(4)),
    target_state(Eigen::VectorXd::Zero(9)),
    max_match_distance_(max_match_distance),
    max_match_yaw_diff_(max_match_yaw_diff) {}

void Tracker::Init(const Armors::SharedPtr& armors_msg) {
    if (armors_msg->armors.empty()) {
        return;
    }

    // 简单地选择距离图像中心最近的装甲
    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];
    for (const auto& armor: armors_msg->armors) {
        if (armor.distance_to_image_center < min_distance) {
            min_distance = armor.distance_to_image_center;
            tracked_armor = armor;
        }
    }

    InitEKF(tracked_armor);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

    tracked_id = tracked_armor.number;
    tracker_state = DETECTING;

    UpdateArmorsNum(tracked_armor);
}

void Tracker::Update(const Armors::SharedPtr& armors_msg) {
    // KF 预测
    Eigen::VectorXd ekf_prediction = ekf.Predict();
    RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

    bool matched = false;
    // 如果没有找到匹配的装甲，则使用 KF 预测作为默认目标状态
    target_state = ekf_prediction;

    if (!armors_msg->armors.empty()) {
        Armor same_id_armor;
        int same_id_armors_count = 0;
        auto predicted_position = GetArmorPositionFromState(ekf_prediction);
        double min_position_diff = DBL_MAX;
        double yaw_diff = DBL_MAX;

        // 寻找具有相同 id 的最近的装甲
        for (const auto& armor: armors_msg->armors) {
            if (armor.number == tracked_id) {
                same_id_armor = armor;
                same_id_armors_count++;

                // 具有相同 id 的装甲板的位置
                auto p = armor.pose.position;
                Eigen::Vector3d position_vec(p.x, p.y, p.z);
                double position_diff = (predicted_position - position_vec).norm();
                // 找到最近的装甲
                if (position_diff < min_position_diff) {
                    min_position_diff = position_diff;
                    yaw_diff = abs(OrientationToYaw(armor.pose.orientation) - ekf_prediction(6));
                    tracked_armor = armor;
                }
            }
        }

        // 存储跟踪器信息
        info_position_diff = min_position_diff;
        info_yaw_diff = yaw_diff;

        // 通过位置差和偏航角差判断装甲是否跳变
        if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
            // 找到匹配的装甲
            matched = true;
            auto p = tracked_armor.pose.position;
            // 更新 EKF
            double measured_yaw = OrientationToYaw(tracked_armor.pose.orientation);
            measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
            target_state = ekf.Update(measurement);
            RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
        } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
            // 装甲板偏跳变
            HandleArmorJump(same_id_armor);
        } else {
            // 未找到匹配的装甲
            RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
        }
    }

    // 防止半径扩散
    target_state(8) = std::clamp(target_state(8), 0.12, 0.4);
    ekf.SetState(target_state);

    // 跟踪状态机
    if (tracker_state == DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > tracking_thres) {
                detect_count_ = 0;
                tracker_state = TRACKING;
            }
        } else {
            detect_count_ = 0;
            tracker_state = LOST;
        }
    } else if (tracker_state == TRACKING) {
        if (!matched) {
            tracker_state = TEMP_LOST;
            lost_count_++;
        }
    } else if (tracker_state == TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_thres) {
                lost_count_ = 0;
                tracker_state = LOST;
            }
        } else {
            tracker_state = TRACKING;
            lost_count_ = 0;
        }
    }
}

void Tracker::InitEKF(const Armor& a) {
    double xa = a.pose.position.x;
    double ya = a.pose.position.y;
    double za = a.pose.position.z;
    last_yaw_ = 0;
    double yaw = OrientationToYaw(a.pose.orientation);

    // 将初始位置设置为目标后方 0.2 米处
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.26;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    dz = 0, another_r = r;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

    ekf.SetState(target_state);
}

void Tracker::UpdateArmorsNum(const Armor& armor) {
    if (armor.type == "large"
        && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
    {
        tracked_armors_num = ArmorsNum::BALANCE_2;
    } else if (tracked_id == "outpost") {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
    } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
    }
}

void Tracker::HandleArmorJump(const Armor& current_armor) {
    double yaw = OrientationToYaw(current_armor.pose.orientation);
    target_state(6) = yaw;
    UpdateArmorsNum(current_armor);
    // 只有 4 个装甲具有 2 个半径和高度
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
        dz = target_state(4) - current_armor.pose.position.z;
        target_state(4) = current_armor.pose.position.z;
        std::swap(target_state(8), another_r);
    }
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

    // 如果位置差异大于 max_match_distance_，将此情况视为 EKF 发散，重置状态
    auto p = current_armor.pose.position;
    Eigen::Vector3d current_p(p.x, p.y, p.z);
    Eigen::Vector3d infer_p = GetArmorPositionFromState(target_state);
    if ((current_p - infer_p).norm() > max_match_distance_) {
        double r = target_state(8);
        target_state(0) = p.x + r * cos(yaw); // xc
        target_state(1) = 0;                  // vxc
        target_state(2) = p.y + r * sin(yaw); // yc
        target_state(3) = 0;                  // vyc
        target_state(4) = p.z;                // za
        target_state(5) = 0;                  // vza
        RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
    }

    ekf.SetState(target_state);
}

double Tracker::OrientationToYaw(const geometry_msgs::msg::Quaternion& q) {
    // 获取装甲偏航角
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // 使偏航角连续变化（-pi~pi 转换为 -inf~inf）
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}

Eigen::Vector3d Tracker::GetArmorPositionFromState(const Eigen::VectorXd& x) {
    // 计算当前装甲的预测位置
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);
    return Eigen::Vector3d(xa, ya, za);
}

Eigen::Vector3d Tracker::ChooseArmor(const CarState& car_state, const float& shooter_yaw, const float& bullet_speed, const float& flytime_offset) {
    auto&& flytime = std::hypot(car_state.position.x(), car_state.position.y(), car_state.position.z()) / bullet_speed + flytime_offset;

    // 整车预测
    CarState predict_car_state(car_state);
    predict_car_state.position = car_state.position + car_state.velocity * flytime;

    ArmorPosition best_armor;
    best_armor.distance_square = std::numeric_limits<float>::max();
    // float min_yaw_diff = std::numeric_limits<float>::max();

    // 在选装甲板的时候用一个标准的圆更好一点
    // 所以用平均半径计算标准圆上的四块装甲板距离车的距离，而不是当前装甲板的实际半径
    auto&& average_armor_radius = (predict_car_state.r[0] + predict_car_state.r[1]) / 2;
    std::vector<ArmorPosition> armors_position;
    auto armor_number = predict_car_state.armors_num;
    for (int i = 0; i < armor_number; i++) {
        double yaw = predict_car_state.position.w() + i * (2 * M_PI / armor_number);
        armors_position.emplace_back(
            predict_car_state.position.x() - predict_car_state.r[i % 2] * cos(yaw),
            predict_car_state.position.y() - predict_car_state.r[i % 2] * sin(yaw),
            predict_car_state.position.z() + i % 2 * dz,
            yaw,
            pow(predict_car_state.position.x() - average_armor_radius * cos(yaw), 2) + pow(predict_car_state.position.y() - average_armor_radius * sin(yaw), 2)
        );
    }

    for (const auto& armor: armors_position) {
        // 选择距离最近的装甲板
        if (armor.distance_square < best_armor.distance_square) {
            best_armor = armor;
        }

        // 枪管 yaw，更具最小的 yaw 选择最佳装甲板
        // float yaw_diff = armor.yaw - shooter_yaw;
        // if (abs(yaw_diff) < min_yaw_diff) {
        //     best_armor = armor;
        //     min_yaw_diff = yaw_diff;
        // }
    }

    return { best_armor.x, best_armor.y, best_armor.z };
};

} // namespace armor
