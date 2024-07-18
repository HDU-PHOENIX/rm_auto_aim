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

namespace armor {

enum class ArmorsNum {
    NORMAL_4 = 4,
    BALANCE_2 = 2,
    OUTPOST_3 = 3
};

struct ArmorPosition {
    float x;               // 装甲板在世界坐标系下的 x
    float y;               // 装甲板在世界坐标系下的 y
    float z;               // 装甲板在世界坐标系下的 z
    float yaw;             // 装甲板坐标系相对于世界坐标系的 yaw 角
    float distance_square; // 装甲板到小陀螺中心的距离的平方

    ArmorPosition() = default;

    ArmorPosition(float x, float y, float z, float yaw, float distance_square):
        x(x),
        y(y),
        z(z),
        yaw(yaw),
        distance_square(distance_square) {};
};

struct CarState {
    std::string id;
    int armors_num;
    Eigen::Vector4d position;
    Eigen::Vector4d velocity;
    double r[2];
    double dz;
};

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

    // 从状态中获取装甲位置
    Eigen::Vector3d GetArmorPositionFromState(const Eigen::VectorXd& x);

    ExtendedKalmanFilter ekf; // 扩展卡尔曼滤波器

    int tracking_thres; // 追踪帧数阈值
    int lost_thres;     // 丢失帧数的阈值

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

    Eigen::Vector3d ChooseArmor(const CarState& car_state, const float& shooter_yaw, const float& bullet_speed, const float& flytime_offset);

    // 用于存储另一对装甲消息
    double dz, another_r;

private:
    // 初始化扩展卡尔曼滤波器
    void InitEKF(const Armor& a);

    /**
     * @brief 更新装甲数字
     * 
     * @param a 装甲
     */
    void UpdateArmorsNum(const Armor& a);

    // 处理装甲跳变
    void HandleArmorJump(const Armor& a);

    // 将方向转换为偏航角
    double OrientationToYaw(const geometry_msgs::msg::Quaternion& q);

    double max_match_distance_; // 最大匹配距离
    double max_match_yaw_diff_; // 最大匹配偏航角差

    int detect_count_; // 检测计数
    int lost_count_;   // 丢失计数

    double last_yaw_; // 上一次的偏航角
};

} // namespace armor

#endif // ARMOR_PROCESSOR__TRACKER_HPP_
