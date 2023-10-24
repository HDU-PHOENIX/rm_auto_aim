#ifndef SERIAL_NODE_HPP_
#define SERIAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "auto_aim_interfaces/msg/target.hpp"

#include "serial/serial.hpp"
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>

namespace sensor {

class SerialNode : public rclcpp::Node {
public:
    explicit SerialNode(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 初始化下位机串口
     *
     * @return std::unique_ptr<sensor::Serial> Serial 指针
     */
    std::unique_ptr<sensor::Serial> InitSerial();

    uint32_t baud_rate_;
    std::string device_name_;
    drivers::serial_driver::FlowControl flow_control_;
    drivers::serial_driver::Parity parity_;
    drivers::serial_driver::StopBits stop_bits_;

    std::unique_ptr<sensor::Serial> serial_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
};

}

#endif