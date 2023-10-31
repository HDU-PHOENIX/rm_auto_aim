#ifndef SERIAL_NODE_HPP_
#define SERIAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"

#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace sensor {

class SerialNode: public rclcpp::Node {
public:
    explicit SerialNode(const rclcpp::NodeOptions& options);
    /**
     * @brief 串口信息的发布循环
     *
     */
    void LoopForPublish();
    /**
     * @brief 接受到串口信息的回调函数
     */
    void SerialInfoCallback(const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg);

private:
    /**
     * @brief 初始化下位机串口
     *
     * @return std::unique_ptr<sensor::Serial> Serial 指针
     */
    std::unique_ptr<sensor::Serial> InitSerial();

    // 串口相关
    uint32_t baud_rate_ = 115200;              // 波特率
    std::string device_name_ = "/dev/ttyACM0"; // 串口设备名
    drivers::serial_driver::FlowControl flow_control_ = drivers::serial_driver::FlowControl::NONE;
    drivers::serial_driver::Parity parity_ = drivers::serial_driver::Parity::NONE;
    drivers::serial_driver::StopBits stop_bits_ = drivers::serial_driver::StopBits::ONE;

    // 线程相关
    std::thread thread_for_publish_;
    std::atomic<bool> canceled_;

    // 串口
    std::unique_ptr<sensor::Serial> serial_;
    // 串口信息
    auto_aim_interfaces::msg::SerialInfo serial_info_;

    // 串口信息订阅（从自瞄获取向下位机发送的信息）
    rclcpp::Subscription<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_info_sub_;
    // 串口信息发布（向相机节点发送下位机的信息）
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_info_pub_;
};

} // namespace sensor

#endif