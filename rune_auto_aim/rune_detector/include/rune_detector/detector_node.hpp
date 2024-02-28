#ifndef RUNE_DETECTOR__DETECTOR_NODE_HPP_
#define RUNE_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/rune.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"

#include "pnp_solver.hpp"

#include "colors.hpp"
// yolox神经网络
#include "nn.h"

namespace rune {

class RuneDetectorNode: public rclcpp::Node {
public:
    explicit RuneDetectorNode(const rclcpp::NodeOptions& options);

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    void ModeSwitchCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

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
    * @brief 创建标记发布者
    */
    void PublishMarkers();

    /**
    * @brief debug 模式下发布识别到的神符图片
    */
    void PublishImg(cv::Mat& img, const sensor_msgs::msg::Image::SharedPtr& img_msg);

    // debug 模式
    bool debug_;
    bool show_pic;
    image_transport::Publisher debug_img_pub_; //debug

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

    // 可视化标记发布者
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // 相机信息订阅者
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    // 相机中心
    cv::Point2f cam_center_;
    // 相机信息
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    // PnP 解算器
    std::unique_ptr<PnPSolver> pnp_solver_;

    //模型路径
    std::string model_path;

    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    //没有识别到神符时发布的信息
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr no_rune_pub_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_switch_sub_;
};

} // namespace rune

#endif // RUNE_DETECTOR__DETECTOR_NODE_HPP_
