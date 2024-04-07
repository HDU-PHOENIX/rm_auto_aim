#pragma once

// ROS
#include "auto_aim_interfaces/msg/debug_rune.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "communicate/msg/serial_info.hpp"
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pnp_solver.hpp"
#include "tracker.hpp" //添加Tracker类

namespace rune {
// 使用tf2_ros::MessageFilter对自动瞄准接口的Armors消息进行过滤
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Rune>;

class RuneTrackerNode: public rclcpp::Node {
public:
    explicit RuneTrackerNode(const rclcpp::NodeOptions& option);

private:
    // 处理Rune消息的回调函数
    void RunesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr);

    void CreateDebugPublisher(); //创建debug信息发布器

    void PublishDebugInfo(); //发布debug信息

    // 发布标记点函数
    void PublishMarkers(const auto_aim_interfaces::msg::Target& target_msg);

    void InitParams(); //初始化ROS2的declare参数

    bool debug_; //debug标志符

    // PnP 解算器
    std::unique_ptr<PnPSolver> pnp_solver_;

    std::unique_ptr<Tracker> tracker_; //算法封装类

    //使用tf2_ros::MessageFilter对自动瞄准接口的Rune消息进行过滤
    message_filters::Subscriber<auto_aim_interfaces::msg::Rune> runes_sub_;
    std::string target_frame_;                                 //目标坐标系
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;              // tf2 缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_; // tf2 监听器
    std::shared_ptr<tf2_filter> tf2_filter_;

    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_; //向shooter节点发送数据

    //没有识别到神符时发布的信息
    rclcpp::Publisher<communicate::msg::SerialInfo>::SharedPtr no_target_pub_;

    auto_aim_interfaces::msg::Target runes_msg_; //自定义的神符信息

    // 可视化标记发布器
    visualization_msgs::msg::Marker armor_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    //debug信息发布器
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugRune>::SharedPtr debug_pub_;
    //debug信息
    auto_aim_interfaces::msg::DebugRune debug_msg_;

    double chasedelay; //追踪延迟 从launch参数给定
    double bullet_speed;
    int filter_astring_threshold;
    double phase_offset;       //相位差补偿,用于补偿观测到的角速度和滤波后角速度的相位差
    double std_a_, std_yawdd_; //ukf的过程噪声
};

} // namespace rune
