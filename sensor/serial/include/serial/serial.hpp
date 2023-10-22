#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include <rclcpp/logger.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "serial/packet.hpp"

namespace sensor
{


class Serial
{
public:
  explicit Serial(const std::unordered_map<std::string, std::string> & params);

  ~Serial();

  void SendRequest();
  ReceivePacket ReadData();

  void WriteCommand(
    const double & pitch_command, const double & yaw_command, const bool & shoot_command);

private:
  void ResolveParams(const std::unordered_map<std::string, std::string> & params);

  void ReopenPort();

  rclcpp::Logger logger_;

//   std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};
}  // namespace sensor

#endif  // _SERIAL_HPP_
