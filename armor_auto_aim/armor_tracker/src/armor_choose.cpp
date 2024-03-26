#include "Eigen/Core"
#include "vector"

#define BALANCE_ARMOR_ID 4
#define OUTPOST_ARMOR_ID 4

struct ArmorPosition {
    float x;               //装甲板在世界坐标系下的 x
    float y;               //装甲板在世界坐标系下的 y
    float z;               //装甲板在世界坐标系下的 z
    float yaw;             //装甲板坐标系相对于世界坐标系的 yaw 角
    float distance_square; //装甲板到小陀螺的距离的平方

    ArmorPosition() = default;

    ArmorPosition(float x, float y, float z, float yaw):
        x(x),
        y(y),
        z(z),
        yaw(yaw) {
        distance_square = x * x + y * y + z * z;
    }
};

struct CarState {
    double x;
    double y;
    double z;
    double yaw;
    double yaw_speed;
    double r[2];
    double distance;
};

Eigen::Vector3d ChooseArmor(const CarState& car_state, const int& armor_id, const double& another_r) {
    // 小陀螺速度过快，返回车中心
    if (car_state.yaw_speed > 0.5) {
        return Eigen::Vector3d(car_state.x, car_state.y, car_state.z);
    }

    std::vector<ArmorPosition> armors_position;
    ArmorPosition best_armor;
    best_armor.distance_square = 1e9;
    if (armor_id == BALANCE_ARMOR_ID) {
        for (int i = 0; i < 2; i++) {
            double yaw = car_state.yaw + i * M_PI;
            armors_position.emplace_back(
                car_state.x + car_state.r[0] * cos(yaw),
                car_state.y + car_state.r[0] * sin(yaw),
                car_state.z,
                yaw
            );
        }
        // TODO: 平衡装甲板

    } else if (armor_id == OUTPOST_ARMOR_ID) {
        for (int i = 0; i < 3; i++) {
            double yaw = car_state.yaw + i * (2 * M_PI / 3);
            armors_position.emplace_back(
                car_state.x + car_state.r[0] * cos(yaw),
                car_state.y + car_state.r[0] * sin(yaw),
                car_state.z,
                yaw
            );
        }
        // TODO: 前哨站装甲板选择

    } else {
        for (int i = 0; i < 3; i++) {
            double yaw = car_state.yaw + i * (M_PI / 2);
            armors_position.emplace_back(
                car_state.x + car_state.r[0] * cos(yaw),
                car_state.y + car_state.r[0] * sin(yaw),
                car_state.z,
                yaw
            );
        }
    }

    for (auto armor: armors_position) {
        if (armor.distance_square < best_armor.distance_square) {
            best_armor = armor;
        }
    }

    return Eigen::Vector3d(best_armor.x, best_armor.y, best_armor.z);
};
