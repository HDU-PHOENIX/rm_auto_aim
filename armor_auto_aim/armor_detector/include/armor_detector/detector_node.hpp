#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <auto_aim_interfaces/msg/detail/serial_info__struct.hpp>
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
#include "auto_aim_interfaces/msg/serial_info.hpp"
namespace armor {

class ArmorDetectorNode: public rclcpp::Node {
public:
    explicit ArmorDetectorNode(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 接收图片的回调函数
     *        识别装甲板 & PnP 解算
     * 
     * @param img_msg 
     */
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    /**
     * @brief 初始化装甲板识别器，设置识别器参数
     *
     * @return std::unique_ptr<Detector> 识别器指针
     */
    std::unique_ptr<Detector> InitDetector();

    /**
     * @brief 节点内的识别装甲板方法，在此方法内调用 Detector 内的方法
     *
     * @return std::vector<Armor> 识别到的装甲板
     */
    std::vector<Armor> DetectArmors(const sensor_msgs::msg::Image::SharedPtr& img_msg);

    /**
     * @brief 创建 debug 信息发布者
    *  @details 在监听到 debug 参数由 false 变为 true 时调用，创建发布者
     */
    void CreateDebugPublishers();

    /**
     * @brief 销毁 debug 信息发布者
     * @details 在监听到 debug 参数由 true 变为 false 时调用，销毁发布者，不占用资源
     */
    void DestroyDebugPublishers();

    /**
     * @brief 发布 debug 信息
     * @details 对外发布 debug 信息，包括灯条信息、装甲板信息、图像信息
     */
    void PublishDebugInfo(
        cv::Mat& img,
        const sensor_msgs::msg::Image::SharedPtr& img_msg,
        const std::vector<Armor>& armors,
        const double& latency
    );

    /**
     * @brief 初始化 Maker 信息
     */
    void InitMarkers();

    /**
     * @brief 发布 Marker 信息
     */
    void PublishMarkers();

    // 装甲板识别器
    std::unique_ptr<Detector> detector_;

    // 自定义的装甲板信息
    auto_aim_interfaces::msg::Armors armors_msg_;
    // 默认的装甲板信息
    auto_aim_interfaces::msg::Armor default_armor_msg_;
    auto_aim_interfaces::msg::Armors default_armors_msg_;
    // 发布者，发布检测到的装甲板
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // 用于视化的标记信息
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    // 可视化标记发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // 相机信息订阅者
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    // 相机中心
    cv::Point2f cam_center_;
    // 相机信息
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    // PnP 解算器
    std::unique_ptr<PnPSolver> pnp_solver_;
    // pnp 是否迭代
    bool pnp_iterative;

    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // Debug information
    bool debug_;
    bool send_default_armor_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
    image_transport::Publisher gray_mask_pub_;
    image_transport::Publisher color_mask_pub_;
    image_transport::Publisher contour_mask_pub_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;
    image_transport::Publisher result_img_pub_;

    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr no_armor_pub_;
};

} // namespace armor

#endif // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
