
#include <cstdio>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "serial/packet.hpp"
#include "serial/serial.hpp"

namespace sensor {

Serial::Serial(
    uint32_t baud_rate,
    std::string device_name,
    FlowControl flow_control,
    Parity parity,
    StopBits stop_bits
):
    reopen_count_(0),
    send_default_data_flag_(false),
    device_name_(device_name),
    owned_ctx_ { new IoContext(2) },
    serial_driver_ { new drivers::serial_driver::SerialDriver(*owned_ctx_) } {
    // 构造设备配置
    this->device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baud_rate,
        flow_control,
        parity,
        stop_bits
    );

    // 尝试初始化
    try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger("serial_node"),
            "Error creating serial port: %s - %s",
            device_name_.c_str(),
            ex.what()
        );
        if (ReopenPort() == false) {
            send_default_data_flag_ = true;
        }
        RCLCPP_ERROR(rclcpp::get_logger("serial_node"), "in catch");
    }

    // reopen failed
    if (send_default_data_flag_) {
        RCLCPP_WARN(
            rclcpp::get_logger("serial_node"),
            "Failed to reopen port, Set default data recv"
        );
    }
}

Serial::~Serial() {
    serial_driver_->port()->close();

    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void Serial::SendData(DataSend packet) {
    if (Legal(packet)) {
        try {
            serial_driver_->port()->send(ToVector(packet));
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(
                rclcpp::get_logger("serial_node"),
                "Error sending data: %s - %s",
                device_name_.c_str(),
                ex.what()
            );
            ReopenPort();
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("serial_node"), "Invalid packet!");
    }
}

DataRecv Serial::ReadData() {
    // 没连下位机时发送默认数据
    if (this->send_default_data_flag_) {
        // RCLCPP_INFO(rclcpp::get_logger("serial_node"), "Sending default data");
        return this->default_data_recv_;
    }

    try {
        std::vector<uint8_t> data(32);
        serial_driver_->port()->receive(data);
        DataRecv&& packet = FromVector(data);
        if (Legal(packet)) {
            return packet;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("serial_node"), "Invalid packet!");
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger("serial_node"),
            "Error while receiving data: %s",
            ex.what()
        );
        ReopenPort();
    }

    return DataRecv();
}

bool Serial::ReopenPort() {
    reopen_count_++;
    if (reopen_count_ > 5) {
        return false;
    }
    RCLCPP_WARN(rclcpp::get_logger("serial_node"), "Attempting to reopen port");
    try {
        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(rclcpp::get_logger("serial_node"), "Successfully reopened port");
    } catch (const std::exception& ex) {
        RCLCPP_WARN(rclcpp::get_logger("serial_node"), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            return ReopenPort();
        }
    }
    return true;
}

void Serial::SetDefaultDataRecv() {
    this->default_data_recv_.start = 's';
    this->default_data_recv_.color = 'r';
    this->default_data_recv_.mode = 'r';
    this->default_data_recv_.speed = 20;
    this->default_data_recv_.euler[0] = 0;
    this->default_data_recv_.euler[1] = 0;
    this->default_data_recv_.euler[2] = 0;
    this->default_data_recv_.shoot_bool = 0;
    this->default_data_recv_.rune_flag = 0;
    this->default_data_recv_.end = 'e';
}

void Serial::SetDefaultDataRecv(
    char& start,
    char& color,
    char& mode,
    double& speed,
    std::vector<double>& euler,
    char& shootbool,
    char& runeflag,
    char& end
) {
    this->default_data_recv_.start = start;
    this->default_data_recv_.color = color;
    this->default_data_recv_.mode = mode;
    this->default_data_recv_.speed = speed;
    this->default_data_recv_.euler[0] = euler[0];
    this->default_data_recv_.euler[1] = euler[1];
    this->default_data_recv_.euler[2] = euler[2];
    this->default_data_recv_.shoot_bool = shootbool;
    this->default_data_recv_.rune_flag = runeflag;
    this->default_data_recv_.end = end;
}

template<typename DataT>
bool Legal(DataT data) {
    return data.start == 's' && data.end == 'e';
}

} // namespace sensor
