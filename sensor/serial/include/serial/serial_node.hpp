#ifndef SERIAL_SERIAL_NODE_HPP
#define SERIAL_SERIAL_NODE_HPP

#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "serial/control.hpp"
#include "serial/imu.hpp"
#include "serial/serial.hpp"

namespace sensor {

class SerialNode: public rclcpp::Node {
    using IMU = sensor::IMU;
    using Control = sensor::Control;

private:
    template<typename TDevice, typename TIdentify>
    struct Group {
        bool enable;
        std::map<TIdentify, sptr<TDevice>> devices;
    };

public:
    enum class IMUIdentify { Main };
    enum class ControlIdentify { Main };

    struct Data {
        Timestamp timestamp = Timestamp::Now();
        sptr<Typesetter> typesetter = std::make_shared<Typesetter>();
        std::map<IMUIdentify, IMU::DataRecv> imu;
        std::map<ControlIdentify, Control::DataRecv> control;
        Statistic<double> statistic_speed;
    };

    Group<IMU, IMUIdentify> imus;
    Group<Control, ControlIdentify> controls;

protected:
    SerialNode();
    void Update();

private:
    struct {
        bool camera_enable;
        bool imu_enable;
        bool control_enable;
    } config;
    umt::Publisher<sptr<Data>> pub_ { "sensor" };
    double t, dt;
    int i = 0;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp {};
    std::time_t GetTimeStamp() {
        tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now(
        ));
        std::time_t time_stamp = tp.time_since_epoch().count();
        return time_stamp;
    }
};
} // namespace sensor

#endif // SERIAL_SERIAL_NODE_HPP