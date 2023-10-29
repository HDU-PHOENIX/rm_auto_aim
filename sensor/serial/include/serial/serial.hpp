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

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;

class Serial {
public:
    /**
     * @brief Serial 类构造函数
     * 
     * @param node_name 调用这个类的节点名
     * @param baud_rate 波特率
     * @param device_name 串口设备名
     */
    explicit Serial(std::string node_name, uint32_t baud_rate, std::string device_name);

    ~Serial();

    /**
     * @brief 从串口读取数据
     * 
     * @param data_recv 
     */
    void ReadData(DataRecv& data_recv);
    /**
     * @brief 向串口发送数据
     * 
     * @param data_send 
     */
    void SendData(DataSend& data_send);

private:
    /**
     * @brief 重新打开串口
     * 
     */
    void ReopenPort();

    /**
     * @brief 检查数据是否合法
     * 
     * @tparam DataT 数据类型
     * @param data 需要检查的数据
     *
     * @return 数据是否合法
     */
    template<typename DataT>
    bool Legal(DataT data) const {
        return data.start == 's' && data.end == 'e';
    }

    std::string node_name_;   // 调用这个类的节点名
    std::string device_name_; // 串口设备名

    std::unique_ptr<IoContext> owned_ctx_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_; // 串口配置
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;     // 串口驱动

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
};
} // namespace sensor

#endif // _SERIAL_HPP_
