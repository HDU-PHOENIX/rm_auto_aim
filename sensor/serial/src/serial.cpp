
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

Serial::Serial(std::string node_name, uint32_t baud_rate, std::string device_name):
    node_name_(node_name),
    device_name_(device_name),
    owned_ctx_ { new IoContext(2) },
    serial_driver_ { new drivers::serial_driver::SerialDriver(*owned_ctx_) } {
    // 构造设备配置
    this->device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baud_rate,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE
    );

    // 尝试初始化
    try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger(this->node_name_),
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

void Serial::ReadData(DataRecv& data_recv) {
    try {
        std::vector<uint8_t> data(32);
        serial_driver_->port()->receive(data);
        data_recv = FromVector(data);
        if (!this->Legal(data_recv)) {
            RCLCPP_ERROR(rclcpp::get_logger(this->node_name_), "Read invalid data!");
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger(this->node_name_),
            "Error while receiving data: %s",
            ex.what()
        );
        ReopenPort();
    }
}

void Serial::SendData(DataSend& data_send) {
    try {
        std::vector<uint8_t> data = ToVector(data_send);
        if (this->Legal(data_send)) {
            serial_driver_->port()->send(data);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger(this->node_name_), "Not allowed to send invalid data!");
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger(this->node_name_),
            "Error while writing data: %s",
            ex.what()
        );
        ReopenPort();
    }
}

void Serial::ReopenPort() {
    RCLCPP_WARN(rclcpp::get_logger(this->node_name_), "Attempting to reopen port");
    try {
        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(rclcpp::get_logger(this->node_name_), "Successfully reopened port");
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger(this->node_name_),
            "Error while reopening port: %s",
            ex.what()
        );
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            ReopenPort();
        }
    }
}

} // namespace sensor
