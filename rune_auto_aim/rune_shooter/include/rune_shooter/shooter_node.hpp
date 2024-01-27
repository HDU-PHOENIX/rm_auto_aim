#include "auto_aim_interfaces/msg/rune_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rune_shooter/shooter.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace rune {

class RuneShooterNode: public rclcpp::Node {
public:
    explicit RuneShooterNode(const rclcpp::NodeOptions& options);
    void PublishMarkers(const Eigen::Vector3d& shoot_pw, const builtin_interfaces::msg::Time& stamp);

private:
    std::unique_ptr<Shooter> InitShooter();
    std::unique_ptr<Shooter> shooter_;                                                 //发射解算器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;    //发布marker可视化
    rclcpp::Subscription<auto_aim_interfaces::msg::RuneTarget>::SharedPtr target_sub_; //接收tracker的消息
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr shooter_info_pub_;  //解算完成后发给下位机
    //仿真用 发给unity的下位机
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

} // namespace rune
