
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

Serial::Serial():
    Node("serial"),
    owned_ctx_ { new IoContext(2) },
    serial_driver_ { new drivers::serial_driver::SerialDriver(*owned_ctx_) } {
    ResolveParams();

    this->target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "target",
        10,
        [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {}
    );

    // this->declare_parameter("device_name", "/dev/ttyUSB0");
    // this->declare_parameter("flow_control", "NONE");
    // this->declare_parameter("parity", "NONE");
    // this->declare_parameter("stop_bits", "ONE");

    // this->get_parameter("baud_rate", baud_rate);
    // this->get_parameter("device_name", device_name_);
    // this->get_parameter("flow_control", flow_control);
    // this->get_parameter("parity", parity);
    // this->get_parameter("stop_bits", stop_bits);

    try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            this->get_logger(),
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

void Serial::SendRequest() {
    DataSend packet;
    // packet.is_request = true;
    // crc16::appendCRC16CheckSum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

    auto data = ToVector(packet);

    try {
        serial_driver_->port()->send(data);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error sending data: %s - %s",
            device_name_.c_str(),
            ex.what()
        );
        ReopenPort();
    }
}

DataRecv Serial::ReadData() {
    try {
        std::vector<uint8_t> header(1);
        serial_driver_->port()->receive(header);

        if (header[0] == 0x5A) {
            std::vector<uint8_t> data(sizeof(DataRecv) - 1);
            serial_driver_->port()->receive(data);
            DataRecv packet = FromVector(data);
            if (packet.Legal()) {
                return packet;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid packet!");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid header: %02X", header[0]);
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error while receiving data: %s", ex.what());
        ReopenPort();
    }

    return DataRecv();
}

void Serial::WriteCommand() {
    try {
        DataSend packet;

        // TODO: set packet

        std::vector<uint8_t> data = ToVector(packet);
        serial_driver_->port()->send(data);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error while writing data: %s", ex.what());
        ReopenPort();
    }
}

void Serial::ResolveParams() {
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    this->baud_rate = 115200;
    this->flow_control = FlowControl::NONE;
    this->parity = Parity::NONE;
    this->stop_bits = StopBits::ONE;

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        this->baud_rate,
        this->flow_control,
        this->parity,
        this->stop_bits
    );
}

void Serial::ReopenPort() {
    RCLCPP_WARN(this->get_logger(), "Attempting to reopen port");
    try {
        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(this->get_logger(), "Successfully reopened port");
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            ReopenPort();
        }
    }
}

} // namespace sensor
