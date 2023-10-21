#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim {
// 扩展卡尔曼滤波器类
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;

    // 函数类型定义
    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    // 构造函数
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

    // 预测一个时间步
    Eigen::MatrixXd Predict();

    // 根据测量更新估计状态
    Eigen::MatrixXd Update(const Eigen::VectorXd& z);

private:
    // 处理非线性向量函数
    VecVecFunc f;
    // 观测非线性向量函数
    VecVecFunc h;
    // f()的雅可比矩阵
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;
    // h()的雅可比矩阵
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;
    // 过程噪声协方差矩阵
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;
    // 测量噪声协方差矩阵
    VecMatFunc update_R;
    Eigen::MatrixXd R;

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

} // namespace rm_auto_aim

#endif // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
