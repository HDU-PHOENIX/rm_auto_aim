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
    explicit Serial(
        uint32_t baud_rate,
        std::string device_name,
        FlowControl flow_control,
        Parity parity,
        StopBits stop_bits
    );

    ~Serial();

    /**
     * @brief 发送数据
     *
     * @param packet 数据包
     */
    void SendData(DataSend packet);
    /**
     * @brief 读取数据
     *
     * @return DataRecv 数据包
     */
    DataRecv ReadData();

    /**
     * @brief 设置默认的数据包 数据从参数服务器中读取
     */
    void SetDefaultDataRecv(
        char& start,
        char& color,
        char& mode,
        double& speed,
        std::vector<double>& euler,
        char& shootbool,
        char& runeflag,
        char& end
    );

private:
    /**
     * @brief 重新打开串口
     *
     * @return 是否成功
     */
    bool ReopenPort();

    int reopen_count_;            // 重新打开串口的次数
    bool send_default_data_flag_; // 是否发送默认数据
    DataRecv default_data_recv_;  // 默认的数据包

    // 串口相关
    std::string device_name_; // 串口设备名
    std::unique_ptr<IoContext> owned_ctx_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};
} // namespace sensor

#endif // _SERIAL_HPP_
