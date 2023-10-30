#include "serial/serial_node.hpp"

namespace sensor {
SerialNode::SerialNode(const rclcpp::NodeOptions& options): Node("serial_node", options) {
    this->serial_ = InitSerial();
    this->serial_info_pub_ =
        create_publisher<auto_aim_interfaces::msg::SerialInfo>("/serial_info", 2);
    this->serial_info_sub_ = create_subscription<auto_aim_interfaces::msg::SerialInfo>(
        "/target_info",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg) {
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

            auto data = ToVector(packet);
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
    );
    thread_for_publish_ = std::thread(std::bind(&SerialNode::LoopForPublish, this));
}

void SerialNode::LoopForPublish() {
    while (rclcpp::ok() && !canceled_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
    return serial;
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::SerialNode)