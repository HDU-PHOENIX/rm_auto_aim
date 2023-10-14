#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim {

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode(const rclcpp::NodeOptions &options);

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    /**
     * @brief 初始化装甲板识别器，设置识别器参数
     *
     * @return std::unique_ptr<Detector> 识别器指针
     */
    std::unique_ptr<Detector> initDetector();

    /**
     * @brief 识别装甲板
     *
     * @return std::vector<Armor> 识别到的装甲板
     */
    std::vector<Armor>
    detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

    /**
     * @brief debug 模式下发布识别到的装甲板信息
     */
    void createDebugPublishers();
    void destroyDebugPublishers();

    /**
     * @brief 创建发布者
     */
    void publishMarkers();

    // 装甲板识别器
    std::unique_ptr<Detector> detector_;

    // 自定义的装甲板信息
    auto_aim_interfaces::msg::Armors armors_msg_;
    // 发布者，发布检测到的装甲板
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // 用于视化的标记信息
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    // 可视化标记发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            marker_pub_;

    // 相机信息订阅者
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    // 相机中心
    cv::Point2f cam_center_;
    // 相机信息
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    // PnP 解算器
    std::unique_ptr<PnPSolver> pnp_solver_;

    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // Debug information
    bool debug_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr
            lights_data_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr
            armors_data_pub_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;
    image_transport::Publisher result_img_pub_;
};

} // namespace rm_auto_aim

#endif // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
