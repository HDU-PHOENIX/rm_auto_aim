
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
        ReopenPort();
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
    try {
        std::vector<uint8_t> header(32);
        serial_driver_->port()->receive(header);
        DataRecv packet = FromVector(header);
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

void Serial::ReopenPort() {
    RCLCPP_WARN(rclcpp::get_logger("serial_node"), "Attempting to reopen port");
    try {
        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(rclcpp::get_logger("serial_node"), "Successfully reopened port");
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger("serial_node"),
            "Error while reopening port: %s",
            ex.what()
        );
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            ReopenPort();
        }
    }
}

template<typename DataT>
bool Legal(DataT data) {
    return data.start == 's' && data.end == 'e';
}

} // namespace sensor
