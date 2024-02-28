#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shooter/shooter.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace auto_aim {

class ShooterNode: public rclcpp::Node {
public:
    explicit ShooterNode(const rclcpp::NodeOptions& options);
    void PublishMarkers(const Eigen::Vector3d& shoot_pw, const builtin_interfaces::msg::Time& stamp);
    void InitMarker();

private:
    // 射击判断
    void ShootingJudge(auto&& yaw_and_pitch, auto_aim_interfaces::msg::SerialInfo& serial_info, const auto_aim_interfaces::msg::Target::SharedPtr& data);
    std::unique_ptr<Shooter> InitShooter();
    std::unique_ptr<Shooter> shooter_;                                                    //发射解算器
    visualization_msgs::msg::Marker marker;                                               //marker可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;       //发布marker可视化
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;        //接收tracker的消息
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr shooter_info_pub_; //解算完成后发给下位机

    // 消抖阈值
    double yaw_threshold_;
    double pitch_threshold_;

    rclcpp::Time last_shoot_time;
    bool debug_; //debug标志符
};

} // namespace auto_aim
