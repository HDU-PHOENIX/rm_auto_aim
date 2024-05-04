#include "auto_aim_interfaces/msg/target.hpp"
#include "communicate/msg/serial_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shooter/shooter.hpp"
#include <chrono>
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
    void Start();
    void AngleRevise(float& yaw, float& pitch);
    std::unique_ptr<Shooter> InitShooter();

    std::unique_ptr<Shooter> shooter_;                                              // 发射解算器
    visualization_msgs::msg::Marker marker;                                         // marker 可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // 发布 marker可视化
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;  // 接收 tracker 的消息
    rclcpp::Publisher<communicate::msg::SerialInfo>::SharedPtr shooter_info_pub_;   // 解算完成后发给下位机

    bool debug_;                    // debug 标志符
    bool updateflag_;               // 更新标志符
    float target_yaw_and_pitch_[2]; //当前目标 yaw 和 pitch

    communicate::msg::SerialInfo serial_info_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace auto_aim
