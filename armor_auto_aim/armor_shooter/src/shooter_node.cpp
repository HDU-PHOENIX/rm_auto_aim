#include "armor_shooter/shooter_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include <rclcpp/logging.hpp>

namespace armor {

ArmorShooterNode::ArmorShooterNode(const rclcpp::NodeOptions& options):
    Node("armor_shooter", options) {
    RCLCPP_INFO(this->get_logger(), "ArmorShooterNode has been initialized.");
    shooter_ = InitShooter();
    shooter_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialInfo>(
        "/shooter_info",
        rclcpp::SensorDataQoS()
    );
    shooter_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/shooter/marker",
        rclcpp::SensorDataQoS()
    );

    yaw_threshold_ = this->declare_parameter("yaw_threshold", 0.01);
    pitch_threshold_ = this->declare_parameter("pitch_threshold", 0.005);

    shooter_->SetHandOffSet(correction_of_y_, correction_of_z_);

    InitMarker();
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            auto&& yaw_and_pitch = shooter_->DynamicCalcCompensate(
                Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z)
            );

            auto_aim_interfaces::msg::SerialInfo serial_info;
            serial_info.speed = msg->v_yaw;

            yaw_and_pitch[0] = abs(yaw_and_pitch[0]) < yaw_threshold_ ? 0 : yaw_and_pitch[0];
            yaw_and_pitch[1] = abs(yaw_and_pitch[1]) < pitch_threshold_ ? 0 : yaw_and_pitch[1];

            // TODO: 💩💩💩
            if (yaw_and_pitch[0] < 0) {
                yaw_and_pitch[0] = std::max(-0.1, yaw_and_pitch[0]);
            } else {
                yaw_and_pitch[0] = std::min(0.1, yaw_and_pitch[0]);
            }
            if (yaw_and_pitch[1] < 0) {
                yaw_and_pitch[1] = std::max(-0.1, yaw_and_pitch[1]);
            } else {
                yaw_and_pitch[1] = std::min(0.1, yaw_and_pitch[1]);
            }
            if (abs(yaw_and_pitch[0]) < 0.05 && abs(yaw_and_pitch[1]) < 0.05) {
                serial_info.can_shoot.set__data('1');
            } else {
                serial_info.can_shoot.set__data('0');
            }

            serial_info.euler = { static_cast<float>(yaw_and_pitch[0]), 0, 0 };
            serial_info.start.set__data('s');
            serial_info.end.set__data('e');
            serial_info.is_find.set__data('1');
            shooter_info_pub_->publish(std::move(serial_info));

            PublishMarkers(shooter_->GetShootPw(), msg->header.stamp);
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
    auto correction_of_y = declare_parameter("correction_of_y", 0.0, param_desc);
    auto correction_of_z = declare_parameter("correction_of_z", 0.0, param_desc);
    auto stop_error = declare_parameter("stop_error", 0.001);
    auto velocity = declare_parameter("velocity", 25);
    int r_k_iter = declare_parameter("R_K_iter", 60);
    return std::make_unique<Shooter>(
        gravity,
        mode,
        k_of_small,
        k_of_large,
        k_of_light,
        correction_of_y,
        correction_of_z,
        stop_error,
        r_k_iter,
        velocity
    );
}

void ArmorShooterNode::PublishMarkers(const Eigen::Vector3d& shoot_pw, const builtin_interfaces::msg::Time& stamp) {
    shooter_marker_.header.stamp = stamp;
    shooter_marker_.pose.position.x = shoot_pw[0];
    shooter_marker_.pose.position.y = shoot_pw[1];
    shooter_marker_.pose.position.z = shoot_pw[2];
    shooter_marker_pub_->publish(shooter_marker_);
}

void ArmorShooterNode::InitMarker() {
    shooter_marker_.header.frame_id = "shooter";
    shooter_marker_.ns = "armor_shooter";
    shooter_marker_.id = 0;
    shooter_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    shooter_marker_.action = visualization_msgs::msg::Marker::ADD;
    shooter_marker_.pose.orientation.x = 0;
    shooter_marker_.pose.orientation.y = 0;
    shooter_marker_.pose.orientation.z = 0;
    shooter_marker_.pose.orientation.w = 1.0;
    shooter_marker_.scale.x = 0.05;
    shooter_marker_.scale.y = 0.05;
    shooter_marker_.scale.z = 0.05;
    shooter_marker_.color.a = 1.0;
    shooter_marker_.color.r = 0.0;
    shooter_marker_.color.g = 0.0;
    shooter_marker_.color.b = 1.0;
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorShooterNode)