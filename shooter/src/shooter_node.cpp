#include "shooter/shooter_node.hpp"
namespace auto_aim {

ShooterNode::ShooterNode(const rclcpp::NodeOptions& options):
    Node("shooter_node", options) {
    RCLCPP_INFO(this->get_logger(), "ShooterNode has been initialized.");
    shooter_ = InitShooter();
    debug_ = this->declare_parameter("debug", true);
    insert_count_ = 0;
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
        rclcpp::SensorDataQoS()
    );
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "ShooterNode update data");
            shooter_->SetHandOffSet(this->get_parameter("correction_of_y").as_double(), this->get_parameter("correction_of_z").as_double());
            if (!msg->is_find && !msg->tracking) {
                serial_info_.is_find.set__data('0');
                return;
            }
            serial_info_.is_find.set__data('1');
            Eigen::Vector2d yaw_and_pitch = shooter_->DynamicCalcCompensate(Eigen::Vector3d(msg->predict_target.position.x, msg->predict_target.position.y, msg->predict_target.position.z));
            for (auto& euler: yaw_and_pitch) {
                if (euler > M_PI) {
                    euler -= (2 * M_PI);
                } else if (euler < -M_PI) {
                    euler += (2 * M_PI);
                }
            }
            mode_ = msg->mode;
            delay_ = msg->delay;
            rune_shoot_permit_ = msg->can_shoot;
            record_last_last_ = record_last_;
            record_last_.target_yaw_and_pitch[0] = target_yaw_and_pitch_[0] = yaw_and_pitch[0];
            record_last_.target_yaw_and_pitch[1] = target_yaw_and_pitch_[1] = yaw_and_pitch[1]; //记录当前的yaw和pitch
            //记录当前自身车yaw和pitch
            record_last_.now_yaw_and_pitch[0] = now_yaw_and_pitch_[0] = static_cast<float>(msg->origin_yaw_and_pitch[0]);
            record_last_.now_yaw_and_pitch[1] = now_yaw_and_pitch_[1] = static_cast<float>(msg->origin_yaw_and_pitch[1]);
            record_last_.time = this->now();
            updateflag_ = true;
        }

    );
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(5ms, std::bind(&ShooterNode::Start, this));
}

void ShooterNode::Start() {
    // RCLCPP_INFO(this->get_logger(), "ShooterNode is running.");
    if (updateflag_) {
        insert_count_ = 0;
        serial_info_.euler = { static_cast<float>(target_yaw_and_pitch_[0]), static_cast<float>(target_yaw_and_pitch_[1]) };
        updateflag_ = false;
    } else {
        if (record_last_.Empty() || record_last_last_.Empty()) {
            // RCLCPP_INFO(this->get_logger(), "No data in the past two times.");
            //如果过去两次都没有数据，直接跳过
            return;
        }
        if (insert_count_ > 5) {
            record_last_last_.Clear();
            record_last_.Clear();
            insert_count_ = 0;
            return;
        }
        //线性插值
        float yaw_error = record_last_.target_yaw_and_pitch[0] - record_last_last_.now_yaw_and_pitch[0];
        float pitch_error = record_last_.target_yaw_and_pitch[1] - record_last_last_.now_yaw_and_pitch[1];
        float dt = (record_last_.time - record_last_last_.time).seconds(); //时间差
        target_yaw_and_pitch_[0] = record_last_.target_yaw_and_pitch[0] + (yaw_error / dt * (this->now() - record_last_.time).seconds());
        target_yaw_and_pitch_[1] = record_last_.target_yaw_and_pitch[1] + (pitch_error / dt * (this->now() - record_last_.time).seconds());
        AngleRevise(target_yaw_and_pitch_[0], target_yaw_and_pitch_[1]);
        yaw_error = record_last_.now_yaw_and_pitch[0] - record_last_last_.now_yaw_and_pitch[0];
        pitch_error = record_last_.now_yaw_and_pitch[1] - record_last_last_.now_yaw_and_pitch[1];
        now_yaw_and_pitch_[0] = record_last_.now_yaw_and_pitch[0] + (yaw_error / dt * (this->now() - record_last_.time).seconds());
        now_yaw_and_pitch_[1] = record_last_.now_yaw_and_pitch[1] + (pitch_error / dt * (this->now() - record_last_.time).seconds());
        AngleRevise(now_yaw_and_pitch_[0], now_yaw_and_pitch_[1]);
        serial_info_.euler = { static_cast<float>(target_yaw_and_pitch_[0]), static_cast<float>(target_yaw_and_pitch_[1]) };
        insert_count_++;
    }

    ShootingJudge(serial_info_); //射击判断
    shooter_info_pub_->publish(serial_info_);
    if (debug_) {
        PublishMarkers(shooter_->GetShootPw(), this->now());
    }
}

void ShooterNode::ShootingJudge(
    communicate::msg::SerialInfo& serial_info
) {
    serial_info.can_shoot.set__data('0');
    if (mode_) {
        // rune
        //解算欧拉角收敛且tracker算法认为可以射击
        if ((std::hypot(abs(target_yaw_and_pitch_[0] - now_yaw_and_pitch_[0]), abs(target_yaw_and_pitch_[0] - now_yaw_and_pitch_[0])) < 0.05)
            && (this->now() - last_shoot_time).seconds() > this->delay_ && rune_shoot_permit_) {
            //确保子弹不会没有飞到就开下一枪
            serial_info.can_shoot.set__data('1');
            last_shoot_time = this->now();
        }
    } else {
        // armor
        if (abs(target_yaw_and_pitch_[0] - now_yaw_and_pitch_[0]) < 0.03
            && abs(target_yaw_and_pitch_[1] - now_yaw_and_pitch_[1]) < 0.03) {
            serial_info.can_shoot.set__data('1');
        }
    }
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
    auto correction_of_y = declare_parameter("correction_of_y", 0.0, param_desc);
    auto correction_of_z = declare_parameter("correction_of_z", 0.0, param_desc);
    auto stop_error = declare_parameter("stop_error", 0.001);
    auto velocity = declare_parameter("bullet_speed", 25.0);
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
