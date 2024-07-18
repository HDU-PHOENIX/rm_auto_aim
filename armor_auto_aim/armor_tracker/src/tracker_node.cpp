#include "armor_tracker/tracker_node.hpp"
#include "armor_tracker/extended_kalman_filter.hpp"

// STD
#include <chrono>
#include <memory>
#include <vector>

namespace armor {
// ArmorTrackerNode 类的构造函数
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions& options):
    Node("armor_tracker", options) {
    RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

    max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);
    bullet_speed_ = declare_parameter("bullet_speed", 25.0);

    this->shooter_coordinate = declare_parameter("shooter_coordinate", "shooter");
    this->odom_coordinate = declare_parameter("odom_coordinate", "odom");

    // Tracker 参数设置
    double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
    double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
    tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
    lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
    max_v_yaw_ = this->declare_parameter("tracker.max_v_yaw", 0.6);
    flytime_offset_ = this->declare_parameter("tracker.flytime_offset", 0.1);

    tracker_->ekf = this->CreateEKF();

    // 重置追踪器服务
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/tracker/reset",
        [this](
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response
        ) {
            tracker_->tracker_state = Tracker::LOST;
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Tracker reset!");
            return;
        }
    );

    // tf2 相关

    // tf2 buffer & listener 相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // 订阅器和过滤器
    armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
    tf2_filter_ = std::make_shared<tf2_filter>(
        armors_sub_,                        // message_filters subscriber
        *tf2_buffer_,                       // tf2 buffer
        odom_coordinate,                    // frame this filter should attempt to transform to
        100,                                // size of the tf2 cache
        this->get_node_logging_interface(), // node logging interface
        this->get_node_clock_interface(),   // node clock interface
        std::chrono::milliseconds(1)        // timeout
    );
    // 注册回调函数
    tf2_filter_->registerCallback(&ArmorTrackerNode::ArmorsCallback, this);

    // 跟踪信息发布器 odom 系下的装甲板坐标
    info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

    // 跟踪目标消息发布器
    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>("/tracker/target", rclcpp::SensorDataQoS());

    this->InitMarkers();
}

void ArmorTrackerNode::ArmorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg) {
    // 将装甲板位置从相机坐标系转换到 odom（目标坐标系）
    for (auto& armor: armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose = armor.pose;
        try {
            armors_msg->header.frame_id = odom_coordinate;
            armor.pose = tf2_buffer_->transform(ps, odom_coordinate).pose;
        } catch (const tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(get_logger(), "Error while transforming  %s", ex.what());
            return;
        }
    }

    // 去除不符合要求的装甲板
    armors_msg->armors.erase(
        std::remove_if(
            armors_msg->armors.begin(),
            armors_msg->armors.end(),
            [this](const auto_aim_interfaces::msg::Armor& armor) {
                return abs(armor.pose.position.z) > 1.2 // odom 系下的装甲板高度高于车
                    || Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm()
                    > max_armor_distance_; // 装甲板距离车过远
            }
        ),
        armors_msg->armors.end()
    );

    auto_aim_interfaces::msg::TrackerInfo info_msg;
    auto_aim_interfaces::msg::Target target_msg;
    CarState car_state;
    rclcpp::Time time = armors_msg->header.stamp;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = odom_coordinate;

    if (tracker_->tracker_state == Tracker::LOST) {
        // 如果追踪器状态为 LOST，重新初始化追踪器
        tracker_->Init(armors_msg);
        target_msg.tracking = false;
    } else {
        // 追踪器状态不为 LOST

        // 计算和上帧之间的时间间隔
        dt_ = (time - last_time_).seconds();
        tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

        // 更新追踪器
        tracker_->Update(armors_msg);

        // 发布信息
        info_msg.position_diff = tracker_->info_position_diff;
        info_msg.yaw_diff = tracker_->info_yaw_diff;
        info_msg.position.x = tracker_->measurement(0);
        info_msg.position.y = tracker_->measurement(1);
        info_msg.position.z = tracker_->measurement(2);
        info_msg.yaw = tracker_->measurement(3);

        if (tracker_->tracker_state == Tracker::DETECTING) {
            target_msg.tracking = false;
        } else if (tracker_->tracker_state == Tracker::TRACKING || tracker_->tracker_state == Tracker::TEMP_LOST) {
            target_msg.tracking = true;
            // 填充目标消息
            const auto& state = tracker_->target_state;
            car_state = {
                tracker_->tracked_id,
                static_cast<int>(tracker_->tracked_armors_num),
                { state(0), state(2), state(4), state(6) },
                { state(1), state(3), state(5), state(7) },
                { state(8), tracker_->another_r },
                tracker_->dz
            };

            // 获取当前时刻 odom to shooter 的转换
            double roll, pitch, yaw;
            auto transform = tf2_buffer_->lookupTransform(odom_coordinate, shooter_coordinate, tf2::TimePointZero).transform;
            tf2::Matrix3x3(
                tf2::Quaternion(
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                )
            )
                .getRPY(roll, pitch, yaw);

            // 预测装甲板位置
            auto predict_armor_position = tracker_->ChooseArmor(
                car_state,
                yaw,
                bullet_speed_,
                flytime_offset_
            );

            target_msg.predict_target.position.set__x(predict_armor_position.x());
            target_msg.predict_target.position.set__y(predict_armor_position.y());
            target_msg.predict_target.position.set__z(predict_armor_position.z());

            // debug info
            info_msg.v_yaw = state(7);
            info_msg.radius_1 = state(8);
            info_msg.radius_2 = tracker_->another_r;
            info_msg.dz = tracker_->dz;
        }
    }

    last_time_ = time;
    PublishMarkers(target_msg, car_state);
    info_pub_->publish(info_msg);
    target_pub_->publish(target_msg);
}

void ArmorTrackerNode::InitMarkers() {
    // 可视化标记相关
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}
ExtendedKalmanFilter ArmorTrackerNode::CreateEKF() {
    // EKF 参数设置
    // 状态：robot_center_x, v_xc, yc, v_yc, armor_z, v_za, yaw, v_yaw, r
    // 测量：armor_x, armor_y, armor_z, yaw
    // f - 过程函数（运动关系）
    auto f = [this](const Eigen::VectorXd& x) {
        /* 
            x_c  v_xc y_c  v_yc z_c v_zc yaw  v_yaw r
            1,   t,   0,   0,   0,   0,   0,   0,   0,
            0,   1,   0,   0,   0,   0,   0,   0,   0,
            0,   0,   1,   t,   0,   0,   0,   0,   0,
            0,   0,   0,   1,   0,   0,   0,   0,   0,
            0,   0,   0,   0,   1,   t,   0,   0,   0,
            0,   0,   0,   0,   0,   1,   0,   0,   0,
            0,   0,   0,   0,   0,   0,   1,   t,   0,
            0,   0,   0,   0,   0,   0,   0,   1,   0,
            0,   0,   0,   0,   0,   0,   0,   0,   1;
         */
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt_;
        x_new(2) += x(3) * dt_;
        x_new(4) += x(5) * dt_;
        x_new(6) += x(7) * dt_;
        return x_new;
    };
    // J_f - 过程函数的雅可比矩阵
    auto j_f = [this](const Eigen::VectorXd&) {
        Eigen::MatrixXd f(9, 9);
        // clang-format off
        f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
              0,   1,   0,   0,   0,   0,   0,   0,   0,
              0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
              0,   0,   0,   1,   0,   0,   0,   0,   0,
              0,   0,   0,   0,   1,   dt_, 0,   0,   0,
              0,   0,   0,   0,   0,   1,   0,   0,   0,
              0,   0,   0,   0,   0,   0,   1,   dt_, 0,
              0,   0,   0,   0,   0,   0,   0,   1,   0,
              0,   0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };
    // h - 观测函数（从状态量到观测量）
    auto h = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(4);
        double robot_center_x = x(0), yc = x(2), yaw = x(6), r = x(8);
        z(0) = robot_center_x - r * cos(yaw); // armor_x
        z(1) = yc - r * sin(yaw);             // armor_y
        z(2) = x(4);                          // armor_z
        z(3) = x(6);                          // yaw
        return z;
    };
    // J_h - 观测函数的雅可比矩阵
    auto j_h = [](const Eigen::VectorXd& x) {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r = x(8);
        // clang-format off
        //   xc  v_xc  yc  v_yc  zc   v_zc yaw            v_yaw  r
        h << 1,   0,   0,   0,   0,   0,   r*sin(yaw),  0,    -cos(yaw),  // xa
             0,   0,   1,   0,   0,   0,  -r*cos(yaw),  0,    -sin(yaw),  // ya
             0,   0,   0,   0,   1,   0,   0,              0,    0,            // za
             0,   0,   0,   0,   0,   0,   1,              0,    0;            // yaw
        // clang-format on
        return h;
    };
    // update_Q - 过程噪声协方差矩阵
    s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 20.0);
    s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
    s2qr_ = declare_parameter("ekf.sigma2_q_r", 800.0);
    auto u_q = [this]() {
        Eigen::MatrixXd q(9, 9);
        double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //    robot_center_x      v_xc    yc      v_yc    armor_z      v_za    yaw     v_yaw   r
        q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
              q_x_vx, q_vx_vx,0,      0,      0,      0,       0,      0,      0,
              0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
              0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
              0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
              0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
              0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
              0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
              0,      0,      0,      0,      0,      0,      0,      0,      q_r;
        // clang-format on
        return q;
    };
    // update_R - 测量噪声协方差矩阵
    r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
    r_yaw = declare_parameter("ekf.r_yaw", 0.02);
    auto u_r = [this](const Eigen::VectorXd& z) {
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor;
        r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
        return r;
    };
    // P - 估计误差协方差矩阵
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity(); // 单位阵
    return ExtendedKalmanFilter { f, h, j_f, j_h, u_q, u_r, p0 };
}

void ArmorTrackerNode::PublishMarkers(const auto_aim_interfaces::msg::Target& target_msg, const CarState& car_state) {
    position_marker_.header = linear_v_marker_.header = angular_v_marker_.header = armor_marker_.header = target_msg.header;

    const auto& position = car_state.position;
    const auto& velocity = car_state.velocity;
    visualization_msgs::msg::MarkerArray marker_array;
    if (target_msg.tracking) {
        double yaw = position.w(), r1 = car_state.r[0], r2 = car_state.r[1];
        double robot_center_x = position.x(), yc = position.y(), armor_z = position.z();
        double vx = velocity.x(), vy = velocity.y(), vz = velocity.z();
        double dz = car_state.dz;

        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = robot_center_x;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = armor_z + dz / 2;

        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker_.points.emplace_back(arrow_end);

        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += velocity.w() / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
        bool is_current_pair = true;
        size_t a_n = car_state.armors_num;
        geometry_msgs::msg::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // 只有 4 个装甲板有 2 个半径和高度
            if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                p_a.z = armor_z + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            } else {
                r = r1;
                p_a.z = armor_z;
            }
            p_a.x = robot_center_x - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);

            armor_marker_.id = i;
            armor_marker_.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, car_state.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
            armor_marker_.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(armor_marker_);
        }
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

        armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.emplace_back(armor_marker_);
    }

    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_pub_->publish(marker_array);
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorTrackerNode)
