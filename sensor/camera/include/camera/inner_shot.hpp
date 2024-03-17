#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <unistd.h>

#include "auto_aim_interfaces/msg/image.hpp"

namespace sensor {

class InnerShotNode: public rclcpp::Node {
public:
    explicit InnerShotNode();
    ~InnerShotNode();

private:
    void InnerShotCallback(const auto_aim_interfaces::msg::Image::SharedPtr img_msg); //内部录制视频

    std::shared_ptr<cv::VideoWriter> video_writer_;
    //内录视频接收
    rclcpp::Subscription<auto_aim_interfaces::msg::Image>::SharedPtr img_inner_shot_sub_;
};

} // namespace sensor
