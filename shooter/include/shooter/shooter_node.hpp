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
    // 射击判断
    void ShootingJudge(communicate::msg::SerialInfo& serial_info);

    void AngleRevise(float& yaw, float& pitch);
    std::unique_ptr<Shooter> InitShooter();
    std::unique_ptr<Shooter> shooter_;                                              //发射解算器
    visualization_msgs::msg::Marker marker;                                         //marker可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; //发布marker可视化
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;  //接收tracker的消息
    rclcpp::Publisher<communicate::msg::SerialInfo>::SharedPtr shooter_info_pub_;   //解算完成后发给下位机

    // 消抖阈值
    double yaw_threshold_;
    double pitch_threshold_;

    rclcpp::Time last_shoot_time;
    bool debug_; //debug标志符

    communicate::msg::SerialInfo serial_info_;

    float delay_;     //延迟时间
    bool updateflag_; //更新标志符
    bool rune_shoot_permit_;
    struct Record {
        rclcpp::Time time;
        float target_yaw_and_pitch[2] = { 0, 0 };
        float now_yaw_and_pitch[2] = { 0, 0 };
        bool Empty() {
            return target_yaw_and_pitch[0] == 0 && target_yaw_and_pitch[1] == 0;
        }
        void Clear() {
            target_yaw_and_pitch[0] = 0;
            target_yaw_and_pitch[1] = 0;
        }

    } record_last_, record_last_last_; //记录上一次的yaw和pitch
    float target_yaw_and_pitch_[2];    //当前目标yaw和pitch
    float now_yaw_and_pitch_[2];       //当前车自身yaw和pitch
    bool mode_;
    int insert_count_; //连续插值次数
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace auto_aim
