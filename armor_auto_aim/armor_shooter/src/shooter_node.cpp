#include "armor_shooter/shooter_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include <algorithm>
#include <auto_aim_interfaces/msg/detail/serial_info__struct.hpp>
#include <memory>
#include <random>

namespace armor {

ArmorShooterNode::ArmorShooterNode(const rclcpp::NodeOptions& options):
    LifecycleNode("armor_shooter", options) {
    RCLCPP_INFO(this->get_logger(), "ArmorShooterNode has been initialized.");
    shooter_ = InitShooter();
    serial_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialInfo>(
        "/serial_info",
        rclcpp::SensorDataQoS()
    );
}

int ArmorShooterNode::OnActivate() {
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            auto&& yaw_and_pitch = shooter_->DynamicCalcCompensate(Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z));
            //TODO: 考虑做防抖处理
            auto_aim_interfaces::msg::SerialInfo serial_info;
            serial_info.start.data = 's';
            serial_info.end.data = 'e';
            serial_info.is_find.data = 1;
            serial_info.euler[3] = yaw_and_pitch[0];
            serial_info.euler[0] = yaw_and_pitch[1];
            serial_info.origin_euler = { 0 };
            serial_info.distance = msg->position.z;
            serial_info_pub_->publish(std::move(serial_info));
        }
    );
    return 1;
}

int ArmorShooterNode::OnDeactivate() {
    target_sub_.reset();
    return 1;
}

std::unique_ptr<Shooter> ArmorShooterNode::InitShooter() {
    auto gravity = declare_parameter("gravity", 9.781);                        // 重力加速度
    auto mode = declare_parameter("mode", 's');                                // 子弹类型
    auto k_of_small = declare_parameter("k_of_small", 0.01903);                // 小弹丸风阻系数
    auto k_of_big = declare_parameter("k_of_big", 0.000556);                   // 大弹丸风阻系数
    auto k_of_light = declare_parameter("k_of_light", 0.00053);                // 荧光弹丸风阻系数
    auto correction_of_x = declare_parameter("correction_of_x", 0.0);          // yaw轴补偿
    auto correction_of_y = declare_parameter("correction_of_y", 0.0);          // pitch轴补偿
    auto stop_error = declare_parameter("stop_error", 0.001);                  // 停止迭代的最小误差(单位m)
    auto velocity = declare_parameter("velocity", 25);                         // 子弹速度
    auto number_of_iterations = declare_parameter("number_of_iterations", 60); // 龙格库塔法求解落点的迭代次数
    return std::make_unique<Shooter>(
        gravity,
        mode,
        k_of_small,
        k_of_big,
        k_of_light,
        correction_of_x,
        correction_of_y,
        stop_error,
        number_of_iterations,
        velocity
    );
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorShooterNode)
