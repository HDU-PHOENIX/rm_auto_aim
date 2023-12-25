#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "serial/serial.hpp"

namespace sensor {

class SerialForUnity: public rclcpp::Node {
public:
    explicit SerialForUnity(const rclcpp::NodeOptions& options);

    /**
     * @brief 接受到仿真的回调函数
     */
    void SerialInfoCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);

private:
    /**
     * @brief 发布坐标系转换
     * 
     * @param frame_id 当前坐标系
     * @param child_frame_id 子坐标系
     * @param q 四元数
     * @param v 平移向量
     */
    void SendTransform(
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
        const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const tf2::Quaternion& q,
        const tf2::Vector3& v,
        const rclcpp::Time& timestamp
    );

    // 串口信息
    auto_aim_interfaces::msg::SerialInfo serial_info_;

    // 串口信息订阅（数据来自仿真）
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr serial_info_sub_;

    // 用于发布 相机坐标系 到 云台坐标系 的转换的广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_camera2shooter_;
    // 用于发布 云台坐标系 到 odom 的转换的广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_shooter2odom_;
    // 从相机坐标系到云台中心的转换
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_camera2shooter_;
    // 从云台中心到 odom 坐标系的转换（补偿 yaw pitch 轴的转动以保证 odom 系静止）
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_shooter2odom_;

    std::vector<double> camera2shooter_tvec_; // 相机坐标系到枪口坐标系的平移向量
    std::vector<double> shooter2odom_tvec_;   // 枪口坐标系到 odom 坐标系的平移向量
};

} // namespace sensor
