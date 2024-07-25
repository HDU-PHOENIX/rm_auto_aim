#pragma once

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector.hpp"

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "auto_aim_interfaces/msg/ignore_classes.hpp"

namespace armor {
class ArmorDetectorNode: public rclcpp::Node {
public:
    explicit ArmorDetectorNode(const rclcpp::NodeOptions& options);

private:
    std::unique_ptr<Detector> CreateDetector();

    /**
     * @brief 订阅图像消息的回调函数
     *
     * @param msg 图像消息
     */
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief 创建调试信息发布者
     */
    void CreateDebugPublishers();

    /**
     * @brief 销毁调试信息发布者
     */
    void DestroyDebugPublishers();

    /**
     * @brief 更新识别器参数（从参数服务器中读取参数）
     */
    void UpdateDetectorParameters();

    /**
     * @brief 发布调试信息
     * 
     * @param armors 装甲板容器
     * @param header 消息头，一般使用图像消息的 header
     */
    void PublishDebugInfo(const std::vector<Armor>& armors, const std_msgs::msg::Header& header);

    /**
     * @brief 发布装甲板信息
     * 
     * @param armors 装甲板容器
     * @param header 消息头，一般使用图像消息的 header
     */
    void PublishArmors(const std::vector<Armor>& armors, const std_msgs::msg::Header& header);

    geometry_msgs::msg::Pose GetPoseMsg(const ArmorPose& pose) const;

    /**
     * @brief 初始化标记信息
     */
    void InitMarkers();

    bool debug_;      // 是否开启调试模式
    bool show_image_; // 调试图片是否发布图像，仅在调试模式下有效
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

    // 调试信息发布者
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;
    image_transport::Publisher result_img_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr debug_armors_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr debug_lights_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr armor_marker_pub_;

    // 用于可视化的标记信息
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;

    std::unique_ptr<Detector> detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::IgnoreClasses>::SharedPtr ignore_classes_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

    cv::Mat result_image_;
    std::string camera_coordinate_; // 相机坐标系名字
    rclcpp::Time last_publish_time_;
};
} // namespace armor
