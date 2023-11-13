#include "auto_aim_interfaces/msg/rune_target.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rune_shooter/shooter.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace rune {

class RuneShooterNode: public rclcpp::Node {
public:
    explicit RuneShooterNode(const rclcpp::NodeOptions& options);

private:
    std::unique_ptr<Shooter> InitShooter();
    std::unique_ptr<Shooter> shooter_;
    rclcpp::Subscription<auto_aim_interfaces::msg::RuneTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_info_pub_;
};

} // namespace rune
