#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP


#include "camera/mindvision.hpp"
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode: public rclcpp::Node, MindVision {
public:
    explicit CameraNode();
    ~CameraNode() override;
    void Loop();

private:
    cv::Mat frame_;
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    std::thread thread_;
    std::atomic<bool> canceled_;
    cv::VideoCapture capture;
    sensor_msgs::msg::Image::SharedPtr image_msg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    // sensor_msgs::msg::Image::UniquePtr msg;
};

#endif // CAMERA_NODE_HPP