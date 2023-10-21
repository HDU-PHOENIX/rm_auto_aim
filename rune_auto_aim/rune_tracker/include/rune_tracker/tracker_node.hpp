// Copyright 2023 wangchi
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

namespace rune
{
// 使用tf2_ros::MessageFilter对自动瞄准接口的Armors消息进行过滤
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;

class RuneTrackerNode : public rclcpp::Node
{
public:
  explicit RuneTrackerNode(const rclcpp::NodeOptions & options);

private:
  // 处理Rune消息的回调函数
  void runesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr);

  // 发布标记点函数
  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // XOY平面中允许的最大装甲距离
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

  // 使用tf2消息过滤器的订阅器
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // 追踪器信息发布器
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // 发布器
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // 可视化标记发布器
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rune

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
