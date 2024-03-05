#include "rune_tracker/tracker_node.hpp"
#define PNP_ITERATION false
namespace rune {
// RuneTrackerNode类的构造函数
RuneTrackerNode::RuneTrackerNode(const rclcpp::NodeOptions& option):
    Node("rune_tracker_node", option) {
    // 打印信息，表示节点已启动
    RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");
    InitParams();
    tracker_ = std::make_unique<Tracker>(option, 1.5, 1.2, filter_astring_threshold);
    debug_ = this->declare_parameter("debug", false);

    if (debug_) {
        CreateDebugPublisher(); //创建Debug发布器
        // 可视化标记发布器
        // 参见 http://wiki.ros.org/rviz/DisplayTypes/Marker
        armor_marker_.ns = "armors";
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.03; //TODO:x,y,z值还需修改
        armor_marker_.scale.y = 0.23;
        armor_marker_.scale.z = 0.125;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.r = 1.0;
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rune_tracker/marker", 10);
    }
    auto&& camera_matrix = declare_parameter("camera_matrix", std::vector<double> {});
    auto&& distortion_coefficients = declare_parameter("distortion_coefficients", std::vector<double> {});
    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, distortion_coefficients);

    // tf2 buffer & listener 相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // 订阅器和过滤器
    runes_sub_.subscribe(this, "/detector/runes", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "shooter");
    tf2_filter_ = std::make_shared<tf2_filter>(
        runes_sub_,                         // message_filters subscriber
        *tf2_buffer_,                       // tf2 buffer
        target_frame_,                      // frame this filter should attempt to transform to
        100,                                // size of the tf2 cache
        this->get_node_logging_interface(), // node logging interface
        this->get_node_clock_interface(),   // node clock interface
        std::chrono::duration<int>(1)       // timeout
    );
    // 注册回调函数
    tf2_filter_->registerCallback(&RuneTrackerNode::RunesCallback, this);

    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS()
    );
    InitRecord(); //打开txt文件 用于记录卡尔曼滤波曲线和原始曲线
}

// runeCallback函数实现 接收rune_detector发布的rune消息
void RuneTrackerNode::RunesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr) {
    rune_ptr->speed = this->get_parameter("bullet_speed").as_double();
    rune_ptr->chasedelay = this->get_parameter("chasedelay").as_double();
    rune_ptr->phase_offset = this->get_parameter("phase_offset").as_double();
    tracker_->Predict(rune_ptr, runes_msg_, debug_msg_);

    cv::Mat rvec, tvec; //tvec为旋转后的相机坐标系下的坐标
    if (pnp_solver_->SolvePnP(tracker_->GetRotatedRune(), rvec, tvec, PNP_ITERATION)) {
    } else {
        RCLCPP_INFO(this->get_logger(), "rune_tracker solve pnp failed");
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header = rune_ptr->header;
    ps.pose.position.x = runes_msg_.pc.position.x = tvec.at<double>(0);
    ps.pose.position.y = runes_msg_.pc.position.y = tvec.at<double>(1);
    ps.pose.position.z = runes_msg_.pc.position.z = tvec.at<double>(2);
    try {
        // 将装甲板位置从 相机坐标系 转换到 shooter（目标坐标系）
        // 此后装甲板信息中的 armor.pose 为装甲板在 shooter 系中的位置
        runes_msg_.pw.position = tf2_buffer_->transform(ps, target_frame_).pose.position;
    } catch (const tf2::ExtrapolationException& ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming  %s", ex.what());
        return;
    }
    runes_msg_.mode = true; //true 表示符模式 false是装甲板
    target_pub_->publish(runes_msg_);
    if (debug_) {
        PublishMarkers(runes_msg_);
        PublishDebugInfo();
    }
}

void RuneTrackerNode::PublishMarkers(const auto_aim_interfaces::msg::Target& target_msg) {
    armor_marker_.header = target_msg.header;
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.id = 0;
    armor_marker_.pose.position.x = target_msg.pc.position.x;
    armor_marker_.pose.position.y = target_msg.pc.position.y;
    armor_marker_.pose.position.z = target_msg.pc.position.z; //可视化相机坐标系下的预测的装甲板位置
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.emplace_back(armor_marker_);
    armor_marker_.header.frame_id = target_frame_;
    armor_marker_.id = 1;
    armor_marker_.pose.position.x = target_msg.pw.position.x;
    armor_marker_.pose.position.y = target_msg.pw.position.y;
    armor_marker_.pose.position.z = target_msg.pw.position.z; //可视化odom坐标系下的装甲板位置
    marker_array.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array);
}

void RuneTrackerNode::CreateDebugPublisher() {
    debug_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugRune>(
        "/rune_tracker/debug",
        rclcpp::SensorDataQoS()
    );
}
void RuneTrackerNode::PublishDebugInfo() {
    debug_msg_.header.stamp = this->now();
    debug_msg_.delay = delay;
    debug_msg_.rotate_angle = tracker_->GetRotateAngle();
    debug_msg_.phase_offset = phase_offset;
    double* tmp = tracker_->GetFittingPara();
    debug_msg_.a_omega_phi_b[0] = tmp[0];
    debug_msg_.a_omega_phi_b[1] = tmp[1];
    debug_msg_.a_omega_phi_b[2] = tmp[2];
    debug_msg_.a_omega_phi_b[3] = tmp[3];
    debug_pub_->publish(debug_msg_);
}

void RuneTrackerNode::InitParams() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.1;
    param_desc.floating_point_range[0].from_value = 20;
    param_desc.floating_point_range[0].to_value = 30;
    bullet_speed = this->declare_parameter("bullet_speed", 25.0, param_desc);
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.01;
    param_desc.floating_point_range[0].from_value = 0;
    param_desc.floating_point_range[0].to_value = 1;
    chasedelay = this->declare_parameter("chasedelay", 0.0, param_desc); //设置chasedelay的默认值0.0
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.1;
    param_desc.floating_point_range[0].from_value = -3.0;
    param_desc.floating_point_range[0].to_value = 3.0;
    phase_offset = this->declare_parameter("phase_offset", 0.0, param_desc); //设置phaseoffset的默认值0.0
    param_desc.floating_point_range.clear();
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 30;
    filter_astring_threshold = this->declare_parameter("filter_astring_threshold", 0, param_desc);
}

void RuneTrackerNode::InitRecord() {
    omega_file.open("./record/omega.txt"); //用于记录曲线
    if (!omega_file.is_open()) {
        while (true) {
            RCLCPP_INFO(this->get_logger(), "cannot open the omega.txt");
        }
    }
    omega_time.open("./record/omegatime.txt");
    if (!omega_time.is_open()) {
        while (true) {
            RCLCPP_INFO(this->get_logger(), "cannot open the omegatime.txt");
        }
    }
    origin_omega_time.open("./record/origin_omegatime.txt");
    if (!origin_omega_time.is_open()) {
        while (true) {
            RCLCPP_INFO(this->get_logger(), "cannot open the origin_omegatime.txt");
        }
    }
    origin_omega_file.open("./record/origin_omega.txt");
    if (!origin_omega_file.is_open()) {
        while (true) {
            RCLCPP_INFO(this->get_logger(), "cannot open the origin_omega.txt");
        }
    }
}

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// 用class_loader注册组件。
// 这充当一种入口点，允许在将其库加载到运行中的进程时发现组件。
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneTrackerNode)
