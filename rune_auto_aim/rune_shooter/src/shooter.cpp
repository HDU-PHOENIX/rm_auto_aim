#include "rune_shooter/shooter.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rune {
Shooter::Shooter(
    const double& gravity,
    const char& mode,
    const double& kof_of_small,
    const double& kof_of_large,
    const double& correction_of_x,
    const double& correction_of_y,
    const double& stop_error,
    const int& R_K_iter,
    const double& velocity
):
    gravity_(gravity),
    kof_(mode == 's' ? kof_of_small : kof_of_large),
    correction_of_x_(correction_of_x),
    correction_of_y_(correction_of_y),
    stop_error_(stop_error),
    R_K_iter_(R_K_iter),
    velocity_(velocity) {}

Eigen::Vector2d Shooter::DynamicCalcCompensate(Eigen::Vector3d xyz) {
    auto max_iter = 100;
    if (velocity_ == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("shooter_node"), "the velocity_ is 0,place verify your velocity_");
    }

    //TODO:根据陀螺仪安装位置调整距离求解方式
    //降维，坐标系Y轴以垂直向上为正方向
    // auto&& xyz_offset = Eigen::Vector3d{xyz[0], xyz[1] + px / 10, xyz[2] + py / 10};
    xyz = { xyz[0], xyz[1] + correction_of_x_ / 10, xyz[2] + correction_of_y_ / 10 };
    orin_pw_ = xyz;
    auto dist_vertical = xyz[2]; //垂直距离

    auto vertical_tmp = dist_vertical;
    auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
    //std::cout << "dist_vertical :" << dist_vertical << std::endl;
    //std::cout << "dist_horizonal" << dist_horizonal << std::endl;
    auto pitch = atan(dist_vertical / dist_horizonal) * 180 / M_PI;
    auto pitch_new = pitch;
    // auto pitch_offset = 0.0;
    //开始使用龙格库塔法求解弹道补偿
    for (int i = 0; i < max_iter; i++) {
        //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
        //初始化
        // auto x = 0.0; not used
        auto y = 0.0;
        auto p = tan(pitch_new / 180 * M_PI);
        auto u = velocity_ / sqrt(1 + pow(p, 2));
        auto delta_x = dist_horizonal / R_K_iter_;
        for (int j = 0; j < R_K_iter_; j++) {
            auto k1_u = -kof_ * u * sqrt(1 + pow(p, 2));
            auto k1_p = -gravity_ / pow(u, 2);
            auto k1_u_sum = u + k1_u * (delta_x / 2);
            auto k1_p_sum = p + k1_p * (delta_x / 2);

            auto k2_u = -kof_ * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            auto k2_p = -gravity_ / pow(k1_u_sum, 2);
            auto k2_u_sum = u + k2_u * (delta_x / 2);
            auto k2_p_sum = p + k2_p * (delta_x / 2);

            auto k3_u = -kof_ * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            auto k3_p = -gravity_ / pow(k2_u_sum, 2);
            auto k3_u_sum = u + k3_u * (delta_x / 2);
            auto k3_p_sum = p + k3_p * (delta_x / 2);

            auto k4_u = -kof_ * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            auto k4_p = -gravity_ / pow(k3_u_sum, 2);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            // x += delta_x; not used
            y += p * delta_x;
        }
        //评估迭代结果,若小于迭代精度需求则停止迭代
        auto error = dist_vertical - y;
        if (abs(error) <= stop_error_) {
            //Log::Info("error <= stop_error the error is {}", error);
            break;
        } else {
            //Log::Info("error > stop_error the error is {}", error);
            vertical_tmp += error;
            // xyz_tmp[1] -= error;
            pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / M_PI;
        }
    }
    //Log::Info("the fit is over");
    pitch_new = pitch_new / 180 * M_PI;
    double&& yaw = atan(xyz[1] / xyz[0]);
    //yaw = yaw;
    shoot_pw_ = { xyz[0], xyz[1] - correction_of_x_ / 10, vertical_tmp - correction_of_y_ / 10 };

    // shoot_pc = coordinate.RunePwToPc(shoot_pw);
    // shoot_pu = coordinate.RunePcToPu(shoot_pc);
    //Log::Debug("orin_pw:{}", orin_pw);
    //Log::Debug("shoot_pw:{}", shoot_pw);
    return Eigen::Vector2d(yaw, pitch_new * -1); //pitch向上为负
    // return Eigen::Vector2d(yaw + coordinate->yaw, (pitch_new + coordinate->pitch) * -1);  //pitch向上为负
}

} // namespace rune
