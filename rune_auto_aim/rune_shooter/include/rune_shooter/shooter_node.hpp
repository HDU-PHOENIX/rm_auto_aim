#include "auto_aim_interfaces/msg/rune_target.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rune_shooter/shooter.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
    //仿真用 发给unity的下位机
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

} // namespace rune
