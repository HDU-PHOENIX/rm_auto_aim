#include <rclcpp/clock.hpp>

#include "camera/camera_node.hpp"

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    canceled_(false) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");
    // 创建发布者
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS());

    // 创建线程，绑定 Loop 函数
    thread_ = std::thread(std::bind(&CameraNode::Loop, this));
}

CameraNode::~CameraNode() {
    // 等待线程结束
    canceled_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CameraNode::Loop() {
    while (rclcpp::ok() && !canceled_.load()) {
        // 从 MindVision 摄像头获取图像
        start_time = rclcpp::Clock().now();
        if (!this->GetFrame(frame_)) {
            RCLCPP_INFO(this->get_logger(), "get image failed");
            continue;
        }
        end_time = rclcpp::Clock().now();
        RCLCPP_INFO(
            this->get_logger(),
            "GetFrame FPS: %f",
            1.0 / (end_time - start_time).seconds()
        );

        // 创建 Image 消息的 UniquePtr msg
        // 向 msg 中填充图像数据
        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "camera_frame";
        msg->height = frame_->rows;
        msg->width = frame_->cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = 0u;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        msg->data.assign(frame_->datastart, frame_->dataend);

        // 发布消息
        RCLCPP_INFO(
            this->get_logger(),
            "timestamp: %d image address in pub: %p",
            msg->header.stamp.nanosec,
            static_cast<void*>(msg.get())
        );
        this->publisher_->publish(std::move(msg));
        last_publish_time = rclcpp::Clock().now();
        RCLCPP_INFO(this->get_logger(), "Publish FPS: %f", 1.0 / (end_time - start_time).seconds());
    }
}
