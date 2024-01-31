#ifndef SERIAL_NODE_HPP_
#define SERIAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "serial/serial.hpp"

namespace sensor {

class SerialNode: public rclcpp::Node {
public:
    explicit SerialNode(const rclcpp::NodeOptions& options);

    /**
     * @brief 串口信息的发布循环
     *
     */
    void LoopForPublish();

    /**
     * @brief 接受到串口信息的回调函数
     */
    void SerialInfoCallback(const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg);

private:
    /**
     * @brief 初始化下位机串口
     *
     * @return std::unique_ptr<sensor::Serial> Serial 指针
     */
    std::unique_ptr<sensor::Serial> InitSerial();

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
        const rclcpp::Time& timestamp,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const tf2::Quaternion& q,
        const tf2::Vector3& v
    );

    // 线程相关
    std::thread thread_for_publish_;
    std::atomic<bool> canceled_;

    // 串口
    std::unique_ptr<sensor::Serial> serial_;
    // 串口信息
    auto_aim_interfaces::msg::SerialInfo serial_info_;

    // 串口信息订阅（从自瞄获取向下位机发送的信息）
    rclcpp::Subscription<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_info_sub_;
    // 串口信息发布（向相机节点发送下位机的信息）
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_info_pub_;

    // 用于发布 相机坐标系 到 云台坐标系 的转换的广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_shooter2camera_;
    // 用于发布 云台坐标系 到 odom 的转换的广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_odom2shooter_;
    // 从相机坐标系到云台中心的转换
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_shooter2camera_;
    // 从云台中心到 odom 坐标系的转换（补偿 yaw pitch 轴的转动以保证 odom 系静止）
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_odom2shooter_;

    std::vector<double> shooter2camera_tvec_; // 枪口坐标系到相机坐标系的平移向量
    double odom2shooter_r_;                   // 枪口到 odom 坐标系的距离
};

} // namespace sensor

#endif
