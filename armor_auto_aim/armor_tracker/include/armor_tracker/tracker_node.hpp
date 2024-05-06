#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace armor {

using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;

class ArmorTrackerNode: public rclcpp::Node {
public:
    explicit ArmorTrackerNode(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 处理 Armors 消息的回调函数
     *        该函数首先将装甲板坐标从相机坐标系转换到 odom 系，并移除不符合要求的装甲板
     *        然后根据追踪器状态调用 Tracker::Init() or Tracker::Update()
     */
    void ArmorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

    /**
     * @brief 创建扩展卡尔曼滤波器
     * @return ExtendedKalmanFilter
     */
    ExtendedKalmanFilter CreateEKF();

    /**
     * @brief 初始化可视化标记
     */
    void InitMarkers();

    /**
     * @brief 发布 Marker debug 信息
     */
    void PublishMarkers(const auto_aim_interfaces::msg::Target& target_msg, const CarState& car_state);

    // 子弹速度
    double bullet_speed_;
    double flytime_offset_;

    double max_v_yaw_;

    // 装甲板距离车中心（ odom 中心 ）的水平最大距离
    double max_armor_distance_;

    // 上次接收消息的时间
    rclcpp::Time last_time_;
    double dt_;

    // 装甲追踪器
    double s2qxyz_, s2qyaw_, s2qr_;
    double r_xyz_factor, r_yaw;
    double lost_time_thres_;
    std::unique_ptr<Tracker> tracker_;

    // 重置追踪器服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

    // 使用 tf 2消息过滤器的订阅器
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
    std::shared_ptr<tf2_filter> tf2_filter_;

    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

    // 可视化标记发布器
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker armor_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::string shooter_coordinate, odom_coordinate;
};
} // namespace armor

#endif // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
