#include "shooter/shooter_node.hpp"

#define TIMER_CALLBACK

namespace auto_aim {

ShooterNode::ShooterNode(const rclcpp::NodeOptions& options):
    Node("shooter_node", options) {
    RCLCPP_INFO(this->get_logger(), "ShooterNode has been initialized.");
    shooter_ = InitShooter();
    debug_ = this->declare_parameter("debug", true);
    if (debug_) {
        InitMarker();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/shooter/marker",
            rclcpp::SensorDataQoS()
        );
    }

    shooter_info_pub_ = this->create_publisher<communicate::msg::SerialInfo>(
        "/shoot_info/left",
        rclcpp::SensorDataQoS()
    );
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            updateflag_ = true;
            if (!msg->is_find && !msg->tracking) {
                serial_info_.is_find.set__data('0');
                return;
            } else {
                serial_info_.is_find.set__data('1');
            }
            serial_info_.distance = std::hypot(
                msg->predict_target.position.x,
                msg->predict_target.position.y,
                msg->predict_target.position.z
            );
            Eigen::Vector2d yaw_and_pitch = shooter_->DynamicCalcCompensate(Eigen::Vector3d(msg->predict_target.position.x, msg->predict_target.position.y, msg->predict_target.position.z));
            for (auto& euler: yaw_and_pitch) {
                if (euler > M_PI) {
                    euler -= (2 * M_PI);
                } else if (euler < -M_PI) {
                    euler += (2 * M_PI);
                }
            }

            target_yaw_and_pitch_[0] = yaw_and_pitch[0];
            target_yaw_and_pitch_[1] = yaw_and_pitch[1];
#ifndef TIMER_CALLBACK
            serial_info_.euler = { static_cast<float>(target_yaw_and_pitch_[0]), static_cast<float>(target_yaw_and_pitch_[1]) };
            shooter_info_pub_->publish(serial_info_);
#endif
            if (debug_) {
                PublishMarkers(shooter_->GetShootPw(), this->now());
            }
        }

    );
#ifdef TIMER_CALLBACK
    timer_ = this->create_wall_timer(std::chrono::milliseconds(3), std::bind(&ShooterNode::Start, this));
#endif
}

void ShooterNode::Start() {
    // TODO: 可做插值
    serial_info_.euler = { static_cast<float>(target_yaw_and_pitch_[0]), static_cast<float>(target_yaw_and_pitch_[1]) };
    shooter_info_pub_->publish(serial_info_);
}

void ShooterNode::AngleRevise(float& yaw, float& pitch) {
    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2 * M_PI;
    }
    if (pitch > M_PI) {
        pitch -= 2 * M_PI;
    } else if (pitch < -M_PI) {
        pitch += 2 * M_PI;
    }
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
    marker.header.frame_id = "odom";
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
    auto stop_error = declare_parameter("stop_error", 0.001);
    auto velocity = declare_parameter("bullet_speed", 25.0);
    int r_k_iter = declare_parameter("R_K_iter", 60);
    return std::make_unique<Shooter>(
        gravity,
        mode,
        k_of_small,
        k_of_large,
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
