#include "serial/serial_node.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

namespace sensor {
SerialNode::SerialNode(const rclcpp::NodeOptions& options):
    Node("serial_node", options) {
    this->serial_ = InitSerial();
    this->shooter2camera_tvec_ = declare_parameter(
        "shooter2camera_tvec",
        std::vector<double> { 0.0, 0.0, 0.0 }
    );
    this->odom2shooter_r_ = declare_parameter("odom2shooter_r", 0.5);
    this->fix_ = declare_parameter("fix", 0.1);

    broadcaster_shooter2camera_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    broadcaster_odom2shooter_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tfs_shooter2camera_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tfs_odom2shooter_ = std::make_unique<geometry_msgs::msg::TransformStamped>();

    this->serial_info_pub_ = create_publisher<auto_aim_interfaces::msg::SerialInfo>(
        "/serial_info",
        rclcpp::SensorDataQoS()
    );
    this->serial_info_sub_ = create_subscription<auto_aim_interfaces::msg::SerialInfo>(
        "/shooter_info",
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
            "Error creating serial port: %s",
            ex.what()
        );
    };
}

void SerialNode::LoopForPublish() {
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        auto&& package = serial_->ReadData();

        auto yaw = package.euler[0];
        auto pitch = package.euler[2];

        serial_info_.start.data = package.start;
        serial_info_.end.data = package.end;
        serial_info_.color.data = package.color;
        serial_info_.mode.data = package.mode;
        serial_info_.speed = package.speed;
        if (abs(last_euler_[0] - package.euler[0]) < 0.01) {
            yaw_fix_ += last_euler_[0] - package.euler[0];
        }
        serial_info_.euler[0] = package.euler[0] - yaw_fix_ / 1.57 * fix_;
        // serial_info_.euler[0] = package.euler[0];
        serial_info_.euler[1] = package.euler[1];
        serial_info_.euler[2] = package.euler[2]; //(0,1,2) = (yaw,roll,pitch)
        serial_info_.shoot_bool.data = package.shoot_bool;
        serial_info_.rune_flag.data = package.rune_flag;

        // x:red y:green z:blue
        // 发布 odom 到 枪口 的坐标系转换（补偿 yaw pitch 轴的云台转动）
        // 四元数字和欧拉角转换 https://quaternions.online
        SendTransform(
            broadcaster_odom2shooter_,
            tfs_odom2shooter_,
            this->now(),
            "odom",
            "shooter",
            [pitch, yaw]() {
                tf2::Quaternion q;
                q.setRPY(0, pitch, yaw);
                return q;
            }(),
            tf2::Vector3(
                odom2shooter_r_ * cos(pitch) * cos(yaw),
                odom2shooter_r_ * cos(pitch) * sin(yaw),
                odom2shooter_r_ * sin(-pitch)
            )
        );
        // // 发布 枪口 到 相机 的坐标系转换
        SendTransform(
            broadcaster_shooter2camera_,
            tfs_shooter2camera_,
            this->now(),
            "shooter",
            "camera",
            []() {
                tf2::Quaternion q;
                q.setEuler(M_PI_2, 0, -M_PI_2);
                return q;
            }(),
            tf2::Vector3(shooter2camera_tvec_[0], shooter2camera_tvec_[1], shooter2camera_tvec_[2])
        );

        serial_info_pub_->publish(serial_info_);

        last_euler_ = { package.euler[0], package.euler[1], package.euler[2] };
    }
}

std::unique_ptr<sensor::Serial> SerialNode::InitSerial() {
    uint32_t baud_rate = declare_parameter("baud_rate", 115200);
    std::string device_name = declare_parameter("device_name", "/dev/ttyACM0");
    // 默认的串口通信协议
    char default_data_recv_start = declare_parameter("default_data_recv_start", 's');
    char default_data_recv_color = declare_parameter("default_data_recv_color", 'r');
    char default_data_recv_mode = declare_parameter("default_data_recv_mode", 'a');
    auto default_data_recv_speed = declare_parameter("default_data_recv_speed", 0.0);
    auto default_data_recv_euler = declare_parameter("default_data_recv_euler", std::vector<double> { 0.0, 0.0, 0.0 });
    char default_data_recv_shootbool = declare_parameter("default_data_recv_shootbool", 0);
    char default_data_recv_runeflag = declare_parameter("default_data_recv_runeflag", 0);
    char default_data_recv_end = declare_parameter("default_data_recv_end", 'e');

    /*
     * baud_rate 波特率
     * device_name 串口设备名
     * flow_control 流控制
     * parity 奇偶校验
     * stop_bits 停止位
     * 
     */
    auto serial = std::make_unique<sensor::Serial>(
        baud_rate,
        device_name,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE
    );
    serial->SetDefaultDataRecv(
        default_data_recv_start,
        default_data_recv_color,
        default_data_recv_mode,
        default_data_recv_speed,
        default_data_recv_euler,
        default_data_recv_shootbool,
        default_data_recv_runeflag,
        default_data_recv_end
    );

    return serial;
}

/**
 * @brief 发布坐标系转换
 * @param broadcaster 
 * @param tfs 
 * @param timestamp 
 * @param frame_id 
 * @param child_frame_id 
 * @param q 
 * @param v 
 */
void SerialNode::SendTransform(
    const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
    const rclcpp::Time& timestamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const tf2::Quaternion& q,
    const tf2::Vector3& v
) {
    tfs->header.stamp = timestamp;
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