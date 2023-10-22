#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace sensor {
struct ReceivePacket {
    uint8_t header = 0x5A;
    float angular_velocity_x = 0.f;
    float angular_velocity_y = 0.f;
    float angular_velocity_z = 0.f;
    float linear_acceleration_x = 0.f;
    float linear_acceleration_y = 0.f;
    float linear_acceleration_z = 0.f;
    float motor_pitch = 0.f;
    float motor_yaw = 0.f;
    uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket {
    uint8_t header = 0xA5;
    bool is_request = false;
    bool shoot_cmd = false;
    int16_t pitch_command = 0;
    int16_t yaw_command = 0;
    uint16_t checksum = 0;
} __attribute__((packed));

struct DataSend {
    char start; // 开始位           0

    float chassis_vx; // 底盘速度 vx      1 ~ 4
    float chassis_vy; // 底盘速度 vy      5 ~ 8

    int gimbal_mode; // yaw 模式        9 ~ 12
    float pitch; // 云台 pitch 角   13 ~ 16
    float yaw; // 云台 yaw 角     17 ~ 20

    bool fire; // 射击指令        21

    uint32_t a1; // 空闲位          22 ~ 25
    uint32_t a2; // 空闲位          26 ~ 29
    uint8_t a3; // 空闲位          30

    char end; // 结束位          31
};

struct DataRecv {
    char start; // 开始位           0

    float chassis_vx; // 底盘转速 vx      1 ~ 4
    float chassis_vy; // 底盘转速 vy      5 ~ 8

    float delta_yaw_angle; // 偏航角           9 ~ 12
    float displacement_x; // x 方向位移       13 ~ 16
    float displacement_y; // y 方向位移       17 ~ 20

    float pitch_gyro_angle; //                 21 ~ 24
    float yaw_gyro_angle; //                 25 ~ 28

    float location_x; //                 49 ~ 52
    float location_y; //                 53 ~ 56

    uint16_t a1; //                 57 ~ 58
    uint32_t a2; //                 59 ~ 62
    char end; //                 63
};

inline ReceivePacket FromVector(const std::vector<uint8_t>& data) {
    ReceivePacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet) + 1);
    return packet;
}

inline std::vector<uint8_t> ToVector(const SendPacket& data) {
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(
        reinterpret_cast<const uint8_t*>(&data),
        reinterpret_cast<const uint8_t*>(&data) + sizeof(SendPacket),
        packet.begin()
    );
    return packet;
}

} // namespace sensor

#endif // PACKET_HPP_
