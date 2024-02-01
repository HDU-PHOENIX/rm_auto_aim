#include "armor_shooter/shooter.hpp"
#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rclcpp/node.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace armor {

class ArmorShooterNode: public rclcpp::Node {
public:
    explicit ArmorShooterNode(const rclcpp::NodeOptions& options);

private:
    void InitMarker();
    void PublishMarkers(const Eigen::Vector3d& position, const builtin_interfaces::msg::Time& stamp);

    std::unique_ptr<Shooter> InitShooter();
    std::unique_ptr<Shooter> shooter_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr shooter_info_pub_;
    visualization_msgs::msg::Marker shooter_marker_;                                   //marker可视化
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shooter_marker_pub_; //发布marker可视化
};

} // namespace armor
