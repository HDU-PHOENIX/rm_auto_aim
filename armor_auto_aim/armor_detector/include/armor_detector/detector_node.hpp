#pragma once

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector.hpp"

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "auto_aim_interfaces/msg/ignore_classes.hpp"
#include "auto_aim_interfaces/msg/image.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "communicate/msg/serial_info.hpp"

namespace armor {
class ArmorDetectorNode: public rclcpp::Node {
public:
    explicit ArmorDetectorNode(const rclcpp::NodeOptions& options);

private:
    std::unique_ptr<Detector> CreateDetector();

    void ImageCallback(const auto_aim_interfaces::msg::Image::SharedPtr msg);

    void CreateDebugPublishers();

    void DestroyDebugPublishers();

    void UpdateDetectorParameters();

    void PublishDebugInfo(const std::vector<Armor>& armors, const std_msgs::msg::Header& header);

    void PublishArmors(const std::vector<Armor>& armors, const auto_aim_interfaces::msg::Image::SharedPtr& msg);

    void InitMarkers();

    bool debug_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr debug_lights_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr debug_armors_pub_;
    cv::Mat result_image_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;
    image_transport::Publisher result_img_pub_;
    rclcpp::Time last_publish_time_;

    // 用于视化的标记信息
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr armor_marker_pub_;

    int lost_count_;
    std::unique_ptr<Detector> detector_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_info_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::IgnoreClasses>::SharedPtr ignore_classes_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<communicate::msg::SerialInfo>::SharedPtr no_armor_pub_;
};
} // namespace armor
