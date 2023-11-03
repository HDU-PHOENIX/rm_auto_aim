#include "camera/camera_node.hpp"

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options): Node("camera_node", options) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    // 创建发布者
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_detector",
        rclcpp::SensorDataQoS()
    );

    // 创建订阅者
    // serial_info_subscriber_ = this->create_subscription<auto_aim_interfaces::msg::SerialInfo>(
    //     "/serial_info",
    //     rclcpp::SensorDataQoS(),
    //     std::bind(&CameraNode::SerialInfoCallback, this, std::placeholders::_1)
    // );
    start();
}

void CameraNode::start() {
    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void CameraNode::LoopForPublish() {
    while (true) {
        // RCLCPP_INFO(this->get_logger(), "Camera node LoopForPublish start");
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
        // 从 MindVision 摄像头获取图像
        if (!this->GetFrame(frame_)) {
            RCLCPP_ERROR(this->get_logger(), "get image failed");
        }
        // 创建 Image 消息的 UniquePtr msg
        // 向 msg 中填充图像数据
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "camera_frame";
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);
        image_publisher->publish(std::move(image_msg));
    }
}

// void CameraNode::SerialInfoCallback(const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg) {
//     // 从 MindVision 摄像头获取图像
//     if (!this->GetFrame(frame_)) {
//         RCLCPP_ERROR(this->get_logger(), "get image failed");
//     }

//     // 创建 Image 消息的 UniquePtr msg
//     // 向 msg 中填充图像数据
//     sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
//     image_msg->header.stamp = this->now();
//     image_msg->header.frame_id = "camera_frame";
//     image_msg->height = frame_->rows;
//     image_msg->width = frame_->cols;
//     image_msg->encoding = "bgr8";
//     image_msg->is_bigendian = 0u;
//     image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
//     image_msg->data.assign(frame_->datastart, frame_->dataend);

//     // 根据 msg->mode.data 的值，选择发布到哪个话题
//     if (msg->mode.data == 'a') {
//         RCLCPP_INFO(this->get_logger(), "publish image for armor");
//         image_publisher_for_armor_->publish(std::move(image_msg));
//     } else if (msg->mode.data == 'r') {
//         // RCLCPP_INFO(this->get_logger(), "publish image for rune");
//         image_publisher_for_rune_->publish(std::move(image_msg));
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "mode should be a or r but got %c", msg->mode.data);
//     }
// }

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
