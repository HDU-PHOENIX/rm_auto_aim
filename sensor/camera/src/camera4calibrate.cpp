#include "camera/camera4calibrate.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>

namespace sensor {

CameraForCalibrate::CameraForCalibrate(const rclcpp::NodeOptions& options):
    Node("camera_node", options) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    mindvision_ = std::make_shared<MindVision>(ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config");
    mindvision_->SetExposureTime(5000);
    // 创建发布者
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_pub",
        rclcpp::SensorDataQoS().keep_last(2)
    );

    frame_ = std::make_shared<cv::Mat>();
    thread_for_publish_ = std::thread(std::bind(&CameraForCalibrate::LoopForPublish, this));
}

CameraForCalibrate::~CameraForCalibrate() {
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void CameraForCalibrate::GetImg() {
    if (!mindvision_->GetFrame(frame_)) {
        RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
        exit(-1);
    }
}

void CameraForCalibrate::LoopForPublish() {
    while (rclcpp::ok()) {
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        image_msg->header.stamp = this->now();
        this->GetImg();
        image_msg->header.frame_id = "camera";
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);

        image_publisher_->publish(std::move(image_msg));
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraForCalibrate)
