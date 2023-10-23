#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <array>
#include <memory>
#include <string>
#include <thread>
// #include <unordered_map>

#include "auto_aim_interfaces/msg/target.hpp"
#include "serial/packet.hpp"

namespace sensor {

class Serial: public rclcpp::Node {
public:
    explicit Serial();

    ~Serial() override;

    void SendRequest();
    DataRecv ReadData();

    void WriteCommand();

private:
    void ResolveParams();

    void ReopenPort();

    std::unique_ptr<IoContext> owned_ctx_;

    uint32_t baud_rate;
    std::string device_name_;
    drivers::serial_driver::FlowControl flow_control;
    drivers::serial_driver::Parity parity;
    drivers::serial_driver::StopBits stop_bits;

    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
};
} // namespace sensor

#endif // _SERIAL_HPP_
