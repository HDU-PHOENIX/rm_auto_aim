#include "serial/serial_node.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_value.hpp>

namespace sensor {
SerialNode::SerialNode(const rclcpp::NodeOptions& options): Node("serial_node", options) {
    this->InitParameter();
    this->serial_ = InitSerial();

    this->serial_info_pub_ =
        create_publisher<auto_aim_interfaces::msg::SerialInfo>("/serial_info", 2);
    this->serial_info_sub_ = create_subscription<auto_aim_interfaces::msg::SerialInfo>(
        "/target_info",
        rclcpp::SensorDataQoS(),
        std::bind(&SerialNode::SerialInfoCallback, this, std::placeholders::_1)
    );

    thread_for_publish_ = std::thread(std::bind(&SerialNode::LoopForPublish, this));
}

void SerialNode::InitParameter() {
    this->declare_parameter("default_data_recv_start", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("default_data_recv_color", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("default_data_recv_mode", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("default_data_recv_speed", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("default_data_recv_euler", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("default_data_recv_shootbool", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("default_data_recv_runeflag", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("default_data_recv_end", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("baud_rate", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("device_name", rclcpp::PARAMETER_STRING);
    this->declare_parameter("flow_control", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("parity", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("stop_bits", rclcpp::PARAMETER_INTEGER);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("device_name", device_name_);
    this->get_parameter("default_data_recv_start", default_data_recv_start_);
    this->get_parameter("default_data_recv_color", default_data_recv_color_);
    this->get_parameter("default_data_recv_mode", default_data_recv_mode_);
    this->get_parameter("default_data_recv_speed", default_data_recv_speed_);
    this->get_parameter("default_data_recv_euler", default_data_recv_euler_);
    this->get_parameter("default_data_recv_shootbool", default_data_recv_shootbool_);
    this->get_parameter("default_data_recv_runeflag", default_data_recv_runeflag_);
    this->get_parameter("default_data_recv_end", default_data_recv_end_);
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
        serial_info_.euler[2] = package.euler[2];
        serial_info_.shoot_bool.data = package.shoot_bool;
        serial_info_.rune_flag.data = package.rune_flag;

        serial_info_pub_->publish(serial_info_);
    }
}

std::unique_ptr<sensor::Serial> SerialNode::InitSerial() {
    // TODO: 从参数服务器中获取串口参数

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

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::SerialNode)