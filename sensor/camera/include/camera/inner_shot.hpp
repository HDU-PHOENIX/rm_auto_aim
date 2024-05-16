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
namespace sensor {

class InnerShotNode: public rclcpp::Node {
public:
    explicit InnerShotNode();
    ~InnerShotNode();

private:
    void InnerShotCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    std::shared_ptr<cv::VideoWriter> video_writer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_inner_shot_sub_;

    int count_;
    int video_count;
};

} // namespace sensor
