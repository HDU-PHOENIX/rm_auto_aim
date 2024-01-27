#include "map"
#include <Eigen/Dense>

namespace armor {

class Shooter {
public:
    explicit Shooter(
        const double& gravity,
        const char& mode,
        const double& k_of_small,
        const double& k_of_big,
        const double& k_of_light,
        const double& correction_of_x,
        const double& correction_of_y,
        const double& stop_error,
        const int& number_of_iterations,
        const double& velocity
    );
    ~Shooter() = default;

    void Shoot();
    /**
     * @brief 龙格库塔法求解微分方程，取得抬枪补偿(Ps:modify from TUP)
     * @param xyz 目标相对于云台的世界坐标系
     * @param velocity 子弹发射的速度
     * @param mode 子弹类型，有 Small, Larget, Light
     */
    Eigen::Vector2d DynamicCalcCompensate(Eigen::Vector3d xyz);

    /**
     *@brief 设置手动补偿量,px,py的物理含义为相机到摩擦轮出射处的偏移量
    */
    void SetHandOffSet(const double& x, const double& y) {
        correction_of_x_ = x;
        correction_of_y_ = y;
    }
    Eigen::Vector3d shoot_pw_; // 预瞄点的世界坐标下的坐标

private:
    double gravity_;           // 重力系数
    double k_;                 // 风阻系数
    double correction_of_x_;   // yaw轴补偿
    double correction_of_y_;   // pitch轴补偿
    double stop_error_;        // 停止迭代的最小误差(单位m)
    int number_of_iterations_; // 龙格库塔法求解落点的迭代次数
    double velocity_;          // 子弹速度
    Eigen::Vector3d orin_pw_;  // 目标点的世界坐标下的坐标
    Eigen::Vector3d shoot_pc_; // 预瞄点的相机坐标下的坐标
    Eigen::Vector3d shoot_pu_; // 预瞄点的像素坐标下的坐标
};
} // namespace armor
