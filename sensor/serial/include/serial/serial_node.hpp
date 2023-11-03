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

    ~SerialNode();
    /**
   * @brief 接受到串口信息的回调函数
   */
    void SerialInfoCallback(const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg);

    /**
    * @brief 初始化参数服务器,参数服务器用于存储下位机的默认数据和串口信息
     */
    void InitParameter();

private:
    /**
   * @brief 初始化下位机串口
   *
   * @return std::unique_ptr<sensor::Serial> Serial 指针
   */
    std::unique_ptr<sensor::Serial> InitSerial();

    // 串口相关
    // uint32_t baud_rate_ = 115200; // 波特率
    // std::string device_name_ = "/dev/ttyACM0"; // 串口设备名
    drivers::serial_driver::FlowControl flow_control_ = drivers::serial_driver::FlowControl::NONE;
    drivers::serial_driver::Parity parity_ = drivers::serial_driver::Parity::NONE;
    drivers::serial_driver::StopBits stop_bits_ = drivers::serial_driver::StopBits::ONE;
    uint32_t baud_rate_; // 波特率
    std::string device_name_; // 串口设备名

    // 默认下位机数据包信息设置参数
    char default_data_recv_start_;
    char default_data_recv_color_;
    char default_data_recv_mode_;
    double default_data_recv_speed_;
    std::vector<double> default_data_recv_euler_;
    int default_data_recv_shootbool_;
    int default_data_recv_runeflag_;
    char default_data_recv_end_;

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
    // rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_info_pub_;
    // 向符detector_node发送下位机的信息
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_pub_rune;
    // 向armor detector_node发送下位机的信息
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialInfo>::SharedPtr serial_pub_armor;
};

} // namespace sensor

#endif