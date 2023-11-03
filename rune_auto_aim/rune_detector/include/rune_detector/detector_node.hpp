#ifndef RUNE_DETECTOR__DETECTOR_NODE_HPP_
#define RUNE_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// #include "message_filters/subscriber.h"
// #include "message_filters/time_synchronizer.h"

// STD
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/rune.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"
// #include "auto_aim_interfaces/msg/armor.hpp"

#include "pnp_solver.hpp"

#include "colors.hpp"
// yolo神经网络
#include "nn.h"

namespace rune {

class RuneDetectorNode: public rclcpp::Node {
public:
    explicit RuneDetectorNode(const rclcpp::NodeOptions& options);

private:
    // void Callback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);
    // void topic_callback(
    //     const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
    //     const auto_aim_interfaces::msg::SerialInfo::ConstSharedPtr &serial_msg);

    /**
   * @brief 初始化神符识别器，设置识别器参数
   *
   * @return std::unique_ptr<Detector> 识别器指针
   */
    std::shared_ptr<NeuralNetwork> InitDetector();

    /**
   * @brief 识别神符
   *
   * @return std::vector<Rune> 识别到的神符
   */
    bool DetectRunes(const sensor_msgs::msg::Image::SharedPtr& img_msg);

    /**
   * @brief debug 模式下发布识别到的神符信息
   */
    void CreateDebugPublishers();
    void DestroyDebugPublishers();

    /**
   * @brief 创建标记发布者
   */
    void PublishMarkers();

    // struct RuneObject {
    //         cv::Point2f vertices[5];//符叶的五个顶点
    //         cv::Rect_<float> rect;
    //         int cls;
    //         int color;
    //         float prob;
    //         std::vector<cv::Point2f> pts;
    //         //cv::Point2f symbol;
    //     };

    // 神符识别器
    std::shared_ptr<NeuralNetwork> detector_;
    std::vector<rune::NeuralNetwork::RuneObject> objects_;
    double confidence_threshold_; // 神经网络置信度阈值

    // 自定义的神符信息
    auto_aim_interfaces::msg::Rune runes_msg_;
    // 发布者，发布检测到的神符
    rclcpp::Publisher<auto_aim_interfaces::msg::Rune>::SharedPtr runes_pub_;

    // 用于视化的标记信息
    visualization_msgs::msg::Marker rune_marker_;
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

    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> image_sub;
    // message_filters::Subscriber<auto_aim_interfaces::msg::SerialInfo>
    // serial_sub;
    // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
    // auto_aim_interfaces::msg::SerialInfo>> sync_; Debug information bool
    // debug_; std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    // std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    // image_transport::Publisher binary_img_pub_;
    // image_transport::Publisher result_img_pub_;
};

} // namespace rune

#endif // RUNE_DETECTOR__DETECTOR_NODE_HPP_
