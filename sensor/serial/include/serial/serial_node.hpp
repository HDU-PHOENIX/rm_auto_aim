#ifndef SERIAL_NODE_HPP_
#define SERIAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"

#include "auto_aim_interfaces/msg/target.hpp"

#include "auto_aim_interfaces/msg/serial_info.hpp" //自定义ros2消息类型

#include <auto_aim_interfaces/msg/detail/target__struct.hpp>
namespace sensor {

class SerialNode: public rclcpp::Node {
public:
    explicit SerialNode(const rclcpp::NodeOptions& options);
    void LoopForPublish();

private:
    /**
     * @brief 初始化下位机串口
     *
     * @return std::unique_ptr<sensor::Serial> Serial 指针
     */
    std::unique_ptr<sensor::Serial> InitSerial();

    uint32_t baud_rate_ = 115200;
    std::string device_name_ = "/dev/ttyACM0";
    drivers::serial_driver::FlowControl flow_control_ = drivers::serial_driver::FlowControl::NONE;
    drivers::serial_driver::Parity parity_ = drivers::serial_driver::Parity::NONE;
    drivers::serial_driver::StopBits stop_bits_ = drivers::serial_driver::StopBits::ONE;

    // 线程相关
    std::thread thread_for_publish_;
    std::atomic<bool> canceled_;

    std::unique_ptr<sensor::Serial> serial_;
    auto_aim_interfaces::msg::SerialInfo serial_info_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_pub_;
};

} // namespace sensor

#endif