#include "shooter/shooter_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include <rclcpp/logging.hpp>
namespace auto_aim {

ShooterNode::ShooterNode(const rclcpp::NodeOptions& options):
    Node("shooter_node", options) {
    RCLCPP_INFO(this->get_logger(), "ShooterNode has been initialized.");
    shooter_ = InitShooter();
    debug_ = this->declare_parameter("debug", false);
    if (debug_) {
        InitMarker();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/shooter/marker",
            rclcpp::SensorDataQoS()
        );
    }
    last_shoot_time = this->now();
    yaw_threshold_ = this->declare_parameter("yaw_threshold", 0.01);
    pitch_threshold_ = this->declare_parameter("pitch_threshold", 0.005);
    shooter_info_pub_ = this->create_publisher<communicate::msg::SerialInfo>(
        "/shoot_info/left",
        10
    );
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            //接收 shooter 坐标系下的坐标
            shooter_->SetHandOffSet(this->get_parameter("correction_of_y").as_double(), this->get_parameter("correction_of_z").as_double());
            //输入 shooter 坐标系下的坐标 输出 yaw 和 pitch
            auto&& yaw_and_pitch = shooter_->DynamicCalcCompensate(Eigen::Vector3d(msg->pw.position.x, msg->pw.position.y, msg->pw.position.z));
            communicate::msg::SerialInfo serial_info;

            serial_info.speed = msg->v_yaw;
            ShootingJudge(yaw_and_pitch, serial_info, msg);
            serial_info.euler = {
                static_cast<float>(yaw_and_pitch[0]) + msg->yaw_and_pitch[0],
                static_cast<float>(yaw_and_pitch[1]) + msg->yaw_and_pitch[1]
            };
            serial_info.start.set__data('s');
            serial_info.end.set__data('e');
            serial_info.is_find.set__data('1');
            shooter_info_pub_->publish(std::move(serial_info));
            if (debug_) {
                PublishMarkers(shooter_->GetShootPw(), msg->header.stamp);
            }
        }
    );
}

void ShooterNode::ShootingJudge(auto&& yaw_and_pitch, communicate::msg::SerialInfo& serial_info, const auto_aim_interfaces::msg::Target::SharedPtr& data) {
    if (data->mode) {
        if ((sqrt(yaw_and_pitch[0] * yaw_and_pitch[0] + yaw_and_pitch[1] * yaw_and_pitch[1]) < 0.05) && data->can_shoot)
        {
            if ((rclcpp::Time(data->header.stamp) - last_shoot_time).seconds() > data->delay) //确保子弹不会没有飞到就开下一枪
            {
                serial_info.can_shoot.set__data('1');
                last_shoot_time = data->header.stamp;
            }
            serial_info.can_shoot.set__data('0');
        } else {
            serial_info.can_shoot.set__data('0');
        }
        return;
    }
    yaw_and_pitch[0] = abs(yaw_and_pitch[0]) < yaw_threshold_ ? 0 : yaw_and_pitch[0];
    yaw_and_pitch[1] = abs(yaw_and_pitch[1]) < pitch_threshold_ ? 0 : yaw_and_pitch[1];

    yaw_and_pitch[0] = std::clamp(yaw_and_pitch[0], -0.1, 0.1);
    yaw_and_pitch[1] = std::clamp(yaw_and_pitch[1], -0.1, 0.1);
    serial_info.can_shoot.set__data(abs(yaw_and_pitch[0]) < 0.05 && abs(yaw_and_pitch[1]) < 0.05 ? '1' : '0');

    return;
}

void ShooterNode::PublishMarkers(const Eigen::Vector3d& shoot_pw, const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::MarkerArray marker_array;
    marker.header.stamp = stamp;
    marker.pose.position.x = shoot_pw[0];
    marker.pose.position.y = shoot_pw[1];
    marker.pose.position.z = shoot_pw[2];
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

void ShooterNode::InitMarker() {
    marker.header.frame_id = "shooter";
    marker.ns = "shooter";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

std::unique_ptr<Shooter> ShooterNode::InitShooter() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    auto gravity = declare_parameter("gravity", 9.781);
    auto mode = declare_parameter("mode", 's');
    auto k_of_small = declare_parameter("k_of_small", 0.01903);
    auto k_of_large = declare_parameter("k_of_large", 0.000556);
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = -0.5;
    param_desc.floating_point_range[0].to_value = 0.5;
    param_desc.floating_point_range[0].step = 0.001;
    auto correction_of_y = declare_parameter("correction_of_y", 0.0, param_desc);
    auto correction_of_z = declare_parameter("correction_of_z", 0.0, param_desc);
    auto stop_error = declare_parameter("stop_error", 0.001);
    auto velocity = declare_parameter("velocity", 25);
    int r_k_iter = declare_parameter("R_K_iter", 60);
    return std::make_unique<Shooter>(
        gravity,
        mode,
        k_of_small,
        k_of_large,
        correction_of_y,
        correction_of_z,
        stop_error,
        r_k_iter,
        velocity
    );
}

} // namespace auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim::ShooterNode)
