#include "serial/serial_node.hpp"
#include "serial/serial.hpp"
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <memory>
#include <rclcpp/qos.hpp>

namespace sensor {
SerialNode::SerialNode(const rclcpp::NodeOptions& options): Node("serial_node", options) {
    this->serial_ = InitSerial();

    this->target_sub_ = create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS(),
        []() {}
    );
}

std::unique_ptr<sensor::Serial> SerialNode::InitSerial() {
    this->get_parameter("baud_rate", this->baud_rate_);
    this->get_parameter("device_name", this->device_name_);
    this->get_parameter("flow_control", this->flow_control_);
    this->get_parameter("parity", this->parity_);
    this->get_parameter("stop_bits", this->stop_bits_);

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