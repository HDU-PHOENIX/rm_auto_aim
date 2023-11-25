#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace armor {
// 扩展卡尔曼滤波器类
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;

    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    explicit ExtendedKalmanFilter(
        const VecVecFunc& f,
        const VecVecFunc& h,
        const VecMatFunc& j_f,
        const VecMatFunc& j_h,
        const VoidMatFunc& u_q,
        const VecMatFunc& u_r,
        const Eigen::MatrixXd& P0
    );

    // 设置初始状态
    void SetState(const Eigen::VectorXd& x0);

    Eigen::MatrixXd Predict();

    Eigen::MatrixXd Update(const Eigen::VectorXd& z);

private:
    VecVecFunc f;          // 状态转移函数
    VecVecFunc h;          // 观测函数（从状态到观测）
    VecMatFunc jacobian_f; // 状态转移函数的雅可比矩阵
    VecMatFunc jacobian_h; // 观测函数的雅可比矩阵
    Eigen::MatrixXd F;     // 状态转移矩阵
    Eigen::MatrixXd H;     // 观测矩阵
    Eigen::MatrixXd Q;     // 过程噪声协方差矩阵
    Eigen::MatrixXd R;     // 测量噪声协方差矩阵
    VoidMatFunc update_Q;  // 过程噪声协方差矩阵更新函数
    VecMatFunc update_R;   // 测量噪声协方差矩阵更新函数

    // 先验误差估计协方差矩阵
    Eigen::MatrixXd P_pri;
    // 后验误差估计协方差矩阵
    Eigen::MatrixXd P_post;

    // 卡尔曼增益
    Eigen::MatrixXd K;

    // 系统维度
    int n;

    // N维单位矩阵
    Eigen::MatrixXd I;

    // 先验状态
    Eigen::VectorXd x_pri;
    // 后验状态
    Eigen::VectorXd x_post;
};

} // namespace armor

#endif // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
