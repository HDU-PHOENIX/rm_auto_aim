#include "camerainfo/camera_info.hpp"

namespace camerainfo {
CameraInfoNode::CameraInfoNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("camera_info", options) {
    this->declare_parameter("camera_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("distortion", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->get_parameter("camera_matrix", camera_matrix_);
    this->get_parameter("distortion", distortion_coefficients_);

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera_info",
        rclcpp::SensorDataQoS()
    );
    std::copy(camera_matrix_.begin(), camera_matrix_.end(), camera_info_.k.data());
    camera_info_.d = distortion_coefficients_;

    auto&& temp_time = this->now();
    while (this->now() - temp_time < rclcpp::Duration(3, 0)) {
        camera_info_pub_->publish(camera_info_);
    }
}
} // namespace camerainfo

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camerainfo::CameraInfoNode)