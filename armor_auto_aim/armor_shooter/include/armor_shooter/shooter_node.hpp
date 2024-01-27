#include "armor_shooter/shooter.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <auto_aim_interfaces/msg/detail/serial_info__struct.hpp>
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <communicate/msg/detail/control__struct.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace armor {

class ArmorShooterNode: public rclcpp::Node {
public:
    explicit ArmorShooterNode(const rclcpp::NodeOptions& options);

private:
    void PublishMarkers(const Eigen::Vector3d& position, const builtin_interfaces::msg::Time& stamp);

    std::unique_ptr<Shooter> InitShooter();
    std::unique_ptr<Shooter> shooter_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr shooter_info_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; //发布marker可视化
};

} // namespace armor
