#include "rune_shooter/shooter_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include <auto_aim_interfaces/msg/detail/rune_target__struct.hpp>

namespace rune {

RuneShooterNode::RuneShooterNode(const rclcpp::NodeOptions& options):
    Node("rune_shooter", options) {
    RCLCPP_INFO(this->get_logger(), "runeShooterNode has been initialized.");
    shooter_ = InitShooter();
    serial_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialInfo>(
        "/shooter_info",
        rclcpp::SensorDataQoS()
    );
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::RuneTarget>(
        "/RuneTracker2Shooter",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::RuneTarget::SharedPtr msg) {
            //输入odom坐标系下的坐标 输出yaw和pitch
            auto&& YawAndPitch = shooter_->DynamicCalcCompensate(Eigen::Vector3d(msg->pw.position.x, msg->pw.position.y, msg->pw.position.z));
            //TODO: 这里可能要做防抖处理
            auto_aim_interfaces::msg::SerialInfo serial_info;
            serial_info.start.data = 's';
            serial_info.end.data = 'e';
            serial_info.is_find.data = '1';
            serial_info.can_shoot.data = '1';
            serial_info.euler[0] = YawAndPitch[0];  //yaw
            serial_info.euler[2] = -YawAndPitch[1]; //pitch
            serial_info.origin_euler = { 0 };
            serial_info.distance = msg->pw.position.z; //TODO: 这里的距离可能还需要修改
            serial_info_pub_->publish(serial_info);
        }
    );
}

std::unique_ptr<Shooter> RuneShooterNode::InitShooter() {
    auto gravity = declare_parameter("gravity", 9.781);
    auto mode = declare_parameter("mode", 's');
    auto k_of_small = declare_parameter("k_of_small", 0.01903);
    auto k_of_large = declare_parameter("k_of_large", 0.000556);
    auto correction_of_x = declare_parameter("correction_of_x", 0.0);
    auto correction_of_y = declare_parameter("correction_of_y", 0.0);
    auto stop_error = declare_parameter("stop_error", 0.001);
    auto velocity = declare_parameter("velocity", 25);
    int r_k_iter = declare_parameter("R_K_iter", 60);
    return std::make_unique<Shooter>(
        gravity,
        mode,
        k_of_small,
        k_of_large,
        correction_of_x,
        correction_of_y,
        stop_error,
        r_k_iter,
        velocity
    );
}

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneShooterNode)
