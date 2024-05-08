#ifndef CAMERA_FOR_CALIBRATE_HPP
#define CAMERA_FOR_CALIBRATE_HPP

#include "camera/mindvision.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace sensor {

class CameraForCalibrate: public rclcpp::Node {
public:
    explicit CameraForCalibrate(const rclcpp::NodeOptions& options);
    ~CameraForCalibrate() override;

private:
    void LoopForPublish(); //发布图像

    void GetImg(); //获取图像

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    // 原始图像发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::thread thread_for_publish_; //获取图像的线程

    std::shared_ptr<MindVision> mindvision_;
};

} // namespace sensor

#endif // CAMERA_FOR_CALIBRATE_HPP
