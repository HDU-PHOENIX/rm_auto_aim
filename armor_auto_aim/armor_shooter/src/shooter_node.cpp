#include "armor_shooter/shooter_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include <opencv4/opencv2/core/matx.hpp>

namespace armor {

ArmorShooterNode::ArmorShooterNode(const rclcpp::NodeOptions& options):
    Node("armor_shooter", options) {
    RCLCPP_INFO(this->get_logger(), "ArmorShooterNode has been initialized.");
    shooter_ = InitShooter();
    shooter_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialInfo>(
        "/shooter_info",
        rclcpp::SensorDataQoS()
    );
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            shooter_->SetHandOffSet(
                this->get_parameter("correction_of_x").as_double(),
                this->get_parameter("correction_of_y").as_double()
            );
            auto&& yaw_and_pitch = shooter_->DynamicCalcCompensate(
                Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z)
            );
            //TODO: 考虑做防抖处理
            auto_aim_interfaces::msg::SerialInfo serial_info;
            serial_info.speed = msg->v_yaw;
            //TODO: pitch 角度上下正负号得确认
            serial_info.euler = { static_cast<float>(yaw_and_pitch[0]), 0, static_cast<float>(yaw_and_pitch[1]) };
            PublishMarkers(shooter_->shoot_pw_, msg->header.stamp);
            shooter_info_pub_->publish(std::move(serial_info));
        }
    );
}

std::unique_ptr<Shooter> ArmorShooterNode::InitShooter() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    auto gravity = declare_parameter("gravity", 9.781);
    auto mode = declare_parameter("mode", 's');
    auto k_of_small = declare_parameter("k_of_small", 0.01903);
    auto k_of_large = declare_parameter("k_of_large", 0.000556);
    auto k_of_light = declare_parameter("k_of_light", 0.00053);
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = -0.5;
    param_desc.floating_point_range[0].to_value = 0.5;
    param_desc.floating_point_range[0].step = 0.001;
    auto correction_of_x = declare_parameter("correction_of_x", 0.0, param_desc);
    auto correction_of_y = declare_parameter("correction_of_y", 0.0, param_desc);
    auto stop_error = declare_parameter("stop_error", 0.001);
    auto velocity = declare_parameter("velocity", 25);
    int r_k_iter = declare_parameter("R_K_iter", 60);
    return std::make_unique<Shooter>(
        gravity,
        mode,
        k_of_small,
        k_of_large,
        k_of_light,
        correction_of_x,
        correction_of_y,
        stop_error,
        r_k_iter,
        velocity
    );
}

void ArmorShooterNode::PublishMarkers(const Eigen::Vector3d& position, const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "shooter";
    marker.header.stamp = stamp;
    marker.ns = "armor_shooter";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorShooterNode)
