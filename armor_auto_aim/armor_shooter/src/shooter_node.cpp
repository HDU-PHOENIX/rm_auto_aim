#include "armor_shooter/shooter_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include <algorithm>
#include <auto_aim_interfaces/msg/detail/serial_info__struct.hpp>
#include <memory>
#include <random>

namespace armor {

ArmorShooterNode::ArmorShooterNode(const rclcpp::NodeOptions& options):
    Node("armor_shooter", options) {
    RCLCPP_INFO(this->get_logger(), "ArmorShooterNode has been initialized.");
    shooter_ = InitShooter();
    serial_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialInfo>(
        "/serial_info",
        rclcpp::SensorDataQoS()
    );
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            auto position = shooter_->DynamicCalcCompensate(Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z));

            auto_aim_interfaces::msg::SerialInfo serial_info;
            serial_info.start.data = 's';
            serial_info.end.data = 'e';
            serial_info.is_find.data = 1;
            serial_info.euler[3] = position[0];
            serial_info.euler[0] = position[1];
            serial_info.origin_euler = { 0 };
            serial_info.distance = msg->dz;
            serial_info_pub_->publish(serial_info);
        }
    );
}

std::unique_ptr<Shooter> ArmorShooterNode::InitShooter() {
    auto gravity = declare_parameter("gravity", 9.8);
    auto mode = declare_parameter("mode", 's');
    auto kof_of_small = declare_parameter("kof_of_small", 0.01903);
    auto kof_of_large = declare_parameter("kof_of_large", 0.000556);
    auto correction_of_x = declare_parameter("correction_of_x", 0.0);
    auto correction_of_y = declare_parameter("correction_of_y", 0.0);
    auto stop_error = declare_parameter("stop_error", 0.0001);
    auto velocity = declare_parameter("velocity", 0.0);
    int r_k_iter = declare_parameter("R_K_iter", 25);
    return std::make_unique<Shooter>(
        gravity,
        mode,
        kof_of_small,
        kof_of_large,
        correction_of_x,
        correction_of_y,
        stop_error,
        r_k_iter,
        velocity
    );
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorShooterNode)
