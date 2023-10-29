#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace sensor {
struct DataSend {
    char start = 's';        // 0       开始位(s)
    char is_find;            // 1       是否找到目标
    char can_shoot;          // 2       是否可以射击
    float yaw_value;         // 3-6     yaw 偏移量
    float pitch_value;       // 7-10    pitch 偏移量
    float enemy_yaw_speed;   // 11-14   yaw 平移速度
    float enemy_pitch_speed; // 15-18   pitch 平移速度
    float target_distance;   // 19-22   目标距离
    char mode;               // 23      模式
    int id = -1;             // 24-27   ???
    char unused[3] = {};     // 28-30   预留位
    char end;                // 27      结束位
} __attribute__((packed));

struct DataRecv {
    char start = 's';     // 0       开始位(s)
    char color;           // 1       颜色
    char mode;            // 2       模式
    float speed = 20;     // 3-6     速度
    float euler[3] = {};  // 7-18    欧拉角度 (0,1,2) = (yaw,roll,pitch)
    char shoot_bool = 0;  // 19      是否射击
    char rune_flag = 0;   // 20      0为不可激活，1为小符，2为大符
    char unused[10] = {}; // 21-30   预留位
    char end = 'e';       // 31      结束位(e)
} __attribute__((packed));

inline DataRecv FromVector(const std::vector<uint8_t>& data) {
    DataRecv packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet));
    return packet;
}

inline std::vector<uint8_t> ToVector(const DataSend& data) {
    std::vector<uint8_t> packet(sizeof(DataSend));
    std::copy(
        reinterpret_cast<const uint8_t*>(&data),
        reinterpret_cast<const uint8_t*>(&data) + sizeof(DataSend),
        packet.begin()
    );
    return packet;
}

} // namespace sensor

#endif // PACKET_HPP_
