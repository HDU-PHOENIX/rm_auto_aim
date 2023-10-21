#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <cstdio>

#include "camera/mindvision.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode: public rclcpp::Node, MindVision {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;
    void LoopForPublish();

private:
    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    // 线程相关
    std::thread thread_for_publish_;
    std::atomic<bool> canceled_;

    // debug 时间数据
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    rclcpp::Time last_publish_time;

    // 原始图像发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

#endif // CAMERA_NODE_HPP
