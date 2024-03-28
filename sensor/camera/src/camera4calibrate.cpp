#include "camera/camera4calibrate.hpp"
#include "opencv2/opencv.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>

namespace sensor {

CameraForCalibrate::CameraForCalibrate(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    MindVision(ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config") {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

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
    if (!this->GetFrame(frame_)) {
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

        cv::imshow("raw", *frame_);
        cv::waitKey(1);

        image_publisher_->publish(std::move(image_msg));
        // RCLCPP_INFO(this->get_logger(), "publish image");
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraForCalibrate)
