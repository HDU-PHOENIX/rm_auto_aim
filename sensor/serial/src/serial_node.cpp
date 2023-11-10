#include "serial/serial_node.hpp"

namespace sensor {
SerialNode::SerialNode(const rclcpp::NodeOptions& options):
    Node("serial_node", options) {
    this->serial_ = InitSerial();

    broadcaster_camera2gimble_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    broadcaster_gimble2odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tfs_camera2gimble_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tfs_gimble2odom_ = std::make_unique<geometry_msgs::msg::TransformStamped>();

    this->serial_info_pub_ =
        create_publisher<auto_aim_interfaces::msg::SerialInfo>("/serial_info", rclcpp::SensorDataQoS());
    this->serial_info_sub_ = create_subscription<auto_aim_interfaces::msg::SerialInfo>(
        "/target_info",
        rclcpp::SensorDataQoS(),
        std::bind(&SerialNode::SerialInfoCallback, this, std::placeholders::_1)
    );

    thread_for_publish_ = std::thread(std::bind(&SerialNode::LoopForPublish, this));
}

void SerialNode::SerialInfoCallback(const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg) {
    sensor::DataSend packet;
    packet.start = msg->start.data;
    packet.end = msg->end.data;
    packet.mode = msg->mode.data;
    packet.is_find = msg->is_find.data;
    packet.can_shoot = msg->can_shoot.data;
    packet.yaw = msg->euler[0];
    packet.pitch = msg->euler[2];
    packet.origin_yaw = msg->origin_euler[0];
    packet.origin_pitch = msg->origin_euler[2];
    packet.distance = msg->distance;

    try {
        serial_->SendData(packet);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger("serial_node"),
            "Error creating serial port: %s - %s",
            device_name_.c_str(),
            ex.what()
        );
    };
}

void SerialNode::LoopForPublish() {
    while (rclcpp::ok() && !canceled_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // 保证节点的回调函数被调用
        // rclcpp::spin_some(this->get_node_base_interface());

        auto&& package = serial_->ReadData();
        serial_info_.start.data = package.start;
        serial_info_.end.data = package.end;
        serial_info_.color.data = package.color;
        serial_info_.mode.data = package.mode;
        serial_info_.speed = package.speed;
        serial_info_.euler[0] = package.euler[0];
        serial_info_.euler[1] = package.euler[1];
        serial_info_.euler[2] = package.euler[2]; //(0,1,2) = (yaw,roll,pitch)
        serial_info_.shoot_bool.data = package.shoot_bool;
        serial_info_.rune_flag.data = package.rune_flag;

        // x:red y:green z:blue
        // 发布 相机 到 云台中心 的坐标系转换
        SendTransform(
            broadcaster_camera2gimble_,
            tfs_camera2gimble_,
            "camera",
            "gimble",
            // gimble: 云台中心
            // 四元数字和欧拉角转换 https://quaternions.online
            []() {
                tf2::Quaternion q;
                q.setRPY(M_PI_2, -M_PI_2, 0);
                return q;
            }(),
            tf2::Vector3(0, 0, -0.2)
        );
        // 发布 云台中心 到 odom 的坐标系转换（补偿 yaw pitch 轴的云台转动）
        SendTransform(
            broadcaster_gimble2odom_,
            tfs_gimble2odom_,
            "gimble",
            "odom",
            [this]() {
                tf2::Quaternion q;
                q.setRPY(0, -serial_info_.euler[2], -serial_info_.euler[0]);
                return q;
            }(),
            tf2::Vector3(0, 0, 0)
        );

        serial_info_pub_->publish(serial_info_);
    }
}

std::unique_ptr<sensor::Serial> SerialNode::InitSerial() {
    baud_rate_ = declare_parameter("baud_rate", 115200);
    device_name_ = declare_parameter("device_name", "/dev/ttyUSB0");
    default_data_recv_start_ = declare_parameter("default_data_recv_start", 's');
    default_data_recv_color_ = declare_parameter("default_data_recv_color", 'r');
    default_data_recv_mode_ = declare_parameter("default_data_recv_mode", 'a');
    default_data_recv_speed_ = declare_parameter("default_data_recv_speed", 0.0);
    default_data_recv_euler_ = declare_parameter("default_data_recv_euler", std::vector<double> { 0.0, 0.0, 0.0 });
    default_data_recv_shootbool_ = declare_parameter("default_data_recv_shootbool", 0);
    default_data_recv_runeflag_ = declare_parameter("default_data_recv_runeflag", 0);
    default_data_recv_end_ = declare_parameter("default_data_recv_end", 'e');

    auto serial = std::make_unique<sensor::Serial>(
        baud_rate_,
        device_name_,
        flow_control_,
        parity_,
        stop_bits_
    );
    serial->SetDefaultDataRecv(
        default_data_recv_start_,
        default_data_recv_color_,
        default_data_recv_mode_,
        default_data_recv_speed_,
        default_data_recv_euler_,
        default_data_recv_shootbool_,
        default_data_recv_runeflag_,
        default_data_recv_end_
    );

    return serial;
}

void SerialNode::SendTransform(
    const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const tf2::Quaternion& q,
    const tf2::Vector3& v
) {
    tfs->header.stamp = this->now();
    tfs->header.frame_id = frame_id;
    tfs->child_frame_id = child_frame_id;
    tfs->transform.rotation.x = q.getX();
    tfs->transform.rotation.y = q.getY();
    tfs->transform.rotation.z = q.getZ();
    tfs->transform.rotation.w = q.getW();
    tfs->transform.translation.x = v.getX();
    tfs->transform.translation.y = v.getY();
    tfs->transform.translation.z = v.getZ();

    broadcaster->sendTransform(*tfs);
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::SerialNode)