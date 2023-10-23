#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace sensor {
struct DataSend {
    char start = 's'; // 开始位(s)         0
    char is_find; // 是否找到目标       1
    char can_shoot; // 是否可以射击       2

    float yaw_value; // yaw 偏移量        3 ~ 6
    float pitch_value; // pitch 偏移量      7 ~ 10
    float enemy_yaw_speed; // yaw 平移速度      11 ~ 14
    float enemy_pitch_speed; // pitch 平移速度    15 ~ 18
    float target_distance; // 目标距离          19 ~ 22
    char mode; // 模式             23
    int id = -1; // ???             24 - 27
    char unused[3] = {}; // ???             28 - 30
    char end; // 结束位           27

    /**
     * @brief 检查数据是否合法
     *
     * @return 数据是否合法
     */
    bool Legal() {
        return start == 's' && end == 'e';
    }
} __attribute__((packed));

struct DataRecv {
    char start = 's';
    char color;
    char mode;
    float speed = 20;
    float euler[3] = {}; //(0,1,2) = (yaw,roll,pitch)
    char shoot_bool = 0;
    char rune_flag = 0; //0为不可激活，1为小符，2为大符
    char unused[10] = {};
    char end = 'e';

    /**
     * @brief 检查数据是否合法
     *
     * @return 数据是否合法
     */
    bool Legal() const {
        return start == 's' && end == 'e';
    }
} __attribute__((packed));

inline DataRecv FromVector(const std::vector<uint8_t>& data) {
    DataRecv packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet) + 1);
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
