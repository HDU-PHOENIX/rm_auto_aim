#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace sensor {
struct DataSend {
    char start = 's';    // 0         开始位(s)
    char is_find;        // 1         是否找到目标
    char can_shoot;      // 2         是否可以射击
    float yaw;           // 3 ~ 6     yaw 偏移量
    float pitch;         // 7 ~ 10    pitch 偏移量
    float origin_yaw;    // 11 ~ 14   接收到的 yaw
    float origin_pitch;  // 15 ~ 18   接收到的 pitch
    float distance;      // 19 ~ 22   目标距离
    char mode;           // 23        模式 装甲板：a  符：r
    int id = -1;         // 24-27     ？
    char unused[3] = {}; // 28-30     预留位
    char end = 'e';      // 31        结束位

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
    char start = 's';     // 0       开始位
    char color;           // 1       颜色
    char mode;            // 2       模式 装甲板：a  符：r
    float speed = 20;     // 3 - 6   速度
    float euler[3] = {};  // 7-24    欧拉角 (0,1,2) = (yaw,roll,pitch)
    char shoot_bool = 0;  // 25
    char rune_flag = 0;   // 26      0为不可激活，1为小符，2为大符
    char unused[10] = {}; // 27-30
    char end = 'e';       // 31

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
