#include <Eigen/Dense>

#define SMALL 0
#define LARGE 1
#define LIGHT 2

namespace auto_aim {

class Shooter {
public:
    explicit Shooter(
        const double& gravity,
        const char& mode,
        const double& kof_of_small,
        const double& kof_of_large,
        const double& stop_error,
        const int& R_K_iter,
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

    Eigen::Vector3d& GetShootPw() {
        return shoot_pw_;
    }

private:
    double gravity_;     //重力系数
    double kof_;         //风阻系数
    double kof_of_small; //小弹丸风阻系数
    double kof_of_large; //大弹丸风阻系数
    double kof_of_light; //荧光弹丸风阻系数
    double stop_error_;  //停止迭代的最小误差(单位m)
    int R_K_iter_;       //龙格库塔法求解落点的迭代次数
    double velocity_;    //子弹速度
    Eigen::Vector3d orin_pw_;
    Eigen::Vector3d shoot_pw_; //预瞄点的世界坐标下的坐标
    Eigen::Vector3d shoot_pc_; //预瞄点的相机坐标下的坐标
    Eigen::Vector3d shoot_pu_; //预瞄点的像素坐标下的坐标
};
} // namespace auto_aim
