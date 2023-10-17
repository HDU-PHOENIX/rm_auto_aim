#include "camera/camera_node.hpp"

CameraNode::CameraNode():
    Node("camera_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
    canceled_(false) {
    sensor_msgs::msg::Image tt;
    RCLCPP_INFO(this->get_logger(), "camera_node start");
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "11");
    thread_ = std::thread(std::bind(&CameraNode::Loop, this));
}
CameraNode::~CameraNode() {
    canceled_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}
void CameraNode::Loop() {
    // While running...
    RCLCPP_INFO(this->get_logger(), "camera_node Loop start");
    while (rclcpp::ok() && !canceled_.load()) {
        // Capture a frame from OpenCV.

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

        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "camera_frame";
        msg->height = frame_.rows;
        msg->width = frame_.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = 0u;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
        msg->data.assign(frame_.datastart, frame_.dataend);

        RCLCPP_INFO(this->get_logger(), "Publish image address: %p", static_cast<void*>(msg.get()));

        start_time = rclcpp::Clock().now();
        this->publisher_->publish(std::move(msg)); // Publish.
        end_time = rclcpp::Clock().now();
        RCLCPP_INFO(this->get_logger(), "Publish FPS: %f", 1.0 / (end_time - start_time).seconds());
    }
}
