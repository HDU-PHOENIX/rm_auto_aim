#include "serial/serial_node.hpp"
#include "serial/serial.hpp"
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
#include <memory>
#include <rclcpp/qos.hpp>

namespace sensor {
SerialNode::SerialNode(const rclcpp::NodeOptions& options): Node("serial_node", options) {
    this->serial_ = InitSerial();
    this->serial_pub_ = create_publisher<auto_aim_interfaces::msg::SerialInfo>("/serial_info", 2);
    this->target_sub_ = create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::QoS(2),
        [this](const auto_aim_interfaces::msg::Target::ConstSharedPtr& tmp) {
            std::cout << "111" << std::endl;
        } //接收tracker部分的回调函数 还没写好 目前在做测试
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
        serial_pub_->publish(serial_info_);
    }
}

std::unique_ptr<sensor::Serial> SerialNode::InitSerial() {
    // uint32_t baud_rate_;
    // std::string device_name_;
    // drivers::serial_driver::FlowControl flow_control_;
    // drivers::serial_driver::Parity parity_;
    // drivers::serial_driver::StopBits stop_bits_;
    // this->get_parameter<uint32_t>("baud_rate", this->baud_rate_);
    // this->get_parameter<std::string>("device_name", this->device_name_);
    // this->get_parameter<drivers::serial_driver::FlowControl>("flow_control", this->flow_control_);
    // this->get_parameter<drivers::serial_driver::Parity>("parity", this->parity_);
    // this->get_parameter<drivers::serial_driver::StopBits>("stop_bits", this->stop_bits_);

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