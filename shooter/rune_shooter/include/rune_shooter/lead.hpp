#pragma once
#include <Eigen/Eigen>
#include <phoenix/utilities/logger.hpp>

namespace phoenix {
    /**
     * @brief 用于将提前量(此处提前量的含义包含提前量原意和抬枪补偿)解算为云台转动角度的类
     * @author Exia
    */
    class Lead {
    public:
        Eigen::Vector3d shoot_pw;    //预瞄点的世界坐标
        Eigen::Vector3d d_shoot_pw;  //识别到的点增加抬枪补偿后的世界坐标
        Eigen::Vector3d shoot_pc;
        Eigen::Vector3d shoot_pu;
        Eigen::Vector3d orin_pw;
        Eigen::Vector3d orin_pc;
        Eigen::Vector3d orin_pu;

        /**
             * @brief 构造函数设置现实环境的物理量，也可使用没有参数传入的构造函数，会有默认值设置
             * @param gravity 重力加速度常数
             * @param kofsmall 小弹丸风阻系数
             * @param koflarge 大弹丸风阻系数
             * @param koflight 荧光弹丸风阻系数
             * @param R_K_iter 龙格库塔法最大迭代次数
             * @param stop_error 停止迭代的最小误差(单位m)
            */
        Lead(const double& gravity, const double& kofsmall, const float& koflarge, const double& koflight, const double& R_K_iter, const double& stop_error)
            : gravity(gravity), kofsmall(kofsmall), koflarge(koflarge), koflight(koflight),
              R_K_iter(R_K_iter), stop_error(stop_error){};

        /**
             * @brief 使用默认值
            */
        Lead() {
            gravity = 9.781;
            kofsmall = 0.01903;
            koflarge = 0.000556;
            koflight = 0.00053;
            R_K_iter = 60;
            stop_error = 0.001;
        }

        ~Lead(){};

        /**
             * @brief 龙格库塔法求解微分方程，取得抬枪补偿(Ps:modify from TUP)
             * @param xyz 目标相对于云台的世界坐标系
             * @param velocity 子弹发射的速度
             * @param mode 子弹类型，有 Bullet::Small,Bullet::Larget,Bullet::Light
            */
        Eigen::Vector2d DynamicCalcCompensate(Eigen::Vector3d& xyz, const float& velocity, const Bullet& mode, Coordinate& coordinate) {
            max_iter = 100;
            if (velocity == 0) {
                Log::Error("the velocity is 0,place verify your velocity");
            }
            switch (mode) {
                case Bullet::Small:
                    k = kofsmall;
                    break;
                case Bullet::Large:
                    k = koflarge;
                    break;
                case Bullet::Light:
                    k = koflight;
                    break;
                default:
                    Log::Error("place select your bullet type first");
            }
            //TODO:根据陀螺仪安装位置调整距离求解方式
            //降维，坐标系Y轴以垂直向上为正方向
            // auto&& xyz_offset = Eigen::Vector3d{xyz[0], xyz[1] + px / 10, xyz[2] + py / 10};
            xyz = {xyz[0], xyz[1] + px / 10, xyz[2] + py / 10};
            orin_pw = {xyz[0], xyz[1], xyz[2]};
            auto dist_vertical = xyz[2];//垂直距离

            auto vertical_tmp = dist_vertical;
            auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
            //std::cout << "dist_vertical :" << dist_vertical << std::endl;
            //std::cout << "dist_horizonal" << dist_horizonal << std::endl;
            auto pitch = atan(dist_vertical / dist_horizonal) * 180 / CV_PI;
            auto pitch_new = pitch;
            // auto pitch_offset = 0.0;
            //开始使用龙格库塔法求解弹道补偿
            for (int i = 0; i < max_iter; i++) {
                //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
                //初始化
                auto x = 0.0;
                auto y = 0.0;
                auto p = tan(pitch_new / 180 * CV_PI);
                auto u = velocity / sqrt(1 + pow(p, 2));
                auto delta_x = dist_horizonal / R_K_iter;
                for (int j = 0; j < R_K_iter; j++) {
                    auto k1_u = -k * u * sqrt(1 + pow(p, 2));
                    auto k1_p = -gravity / pow(u, 2);
                    auto k1_u_sum = u + k1_u * (delta_x / 2);
                    auto k1_p_sum = p + k1_p * (delta_x / 2);

                    auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
                    auto k2_p = -gravity / pow(k1_u_sum, 2);
                    auto k2_u_sum = u + k2_u * (delta_x / 2);
                    auto k2_p_sum = p + k2_p * (delta_x / 2);

                    auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
                    auto k3_p = -gravity / pow(k2_u_sum, 2);
                    auto k3_u_sum = u + k3_u * (delta_x / 2);
                    auto k3_p_sum = p + k3_p * (delta_x / 2);

                    auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
                    auto k4_p = -gravity / pow(k3_u_sum, 2);

                    u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
                    p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

                    x += delta_x;
                    y += p * delta_x;
                }
                //评估迭代结果,若小于迭代精度需求则停止迭代
                auto error = dist_vertical - y;
                if (abs(error) <= stop_error) {
                    //Log::Info("error <= stop_error the error is {}", error);
                    break;
                } else {
                    //Log::Info("error > stop_error the error is {}", error);
                    vertical_tmp += error;
                    // xyz_tmp[1] -= error;
                    pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / CV_PI;
                }
            }
            //Log::Info("the fit is over");
            pitch_new = pitch_new / 180 * M_PI;
            double&& yaw = atan(xyz[1] / xyz[0]);
            //yaw = yaw;
            shoot_pw = {xyz[0], xyz[1] - px / 10, vertical_tmp - py / 10};

            shoot_pc = coordinate.RunePwToPc(shoot_pw);

            shoot_pu = coordinate.RunePcToPu(shoot_pc);
            //Log::Debug("orin_pw:{}", orin_pw);
            //Log::Debug("shoot_pw:{}", shoot_pw);
            return Eigen::Vector2d(yaw, pitch_new * -1);  //pitch向上为负
            // return Eigen::Vector2d(yaw + coordinate->yaw, (pitch_new + coordinate->pitch) * -1);  //pitch向上为负

        }

        /**
             * @brief 项目里原本的自瞄解算方法
             * @param pw 预测后的世界坐标系
             * @param d_pw 识别到目标的世界坐标系
             * @param coordinate 传入已经设置好的coordinate对象
             * @return 返回值为Eigen::Vector4d（dx，dy，d_dx,d_dy) ------ dx,dy为预测后解算为yaw，pitch的值，d_dx,d_dy为没有预测解算的yaw,pitch值
             * @attention coordinate 对象里面的值可能被修改
            */
        Eigen::Vector4d CalOffSet(const Eigen::Vector3d& pw, const Eigen::Vector3d& d_pw, Coordinate& coordinate, const float& v) {
            orin_pw = pw;
            velocity = v;
            auto pred_pw = pw;
            auto d_pred_pw = d_pw;
            double pred_pitch = atan2(pred_pw(2, 0), pred_pw.topRows<2>().norm());
            double d_pred_pitch = atan2(d_pred_pw(2, 0), d_pred_pw.topRows<2>().norm());
            double distance = pred_pw.norm();
            double d_distance = d_pred_pw.norm();
            double a = gravity * gravity * 0.25;
            double d_a = gravity * gravity * 0.25;
            double b = -(double)velocity * velocity - distance * gravity * cos(M_PI_2 + pred_pitch);
            double d_b = -(double)velocity * velocity - d_distance * gravity * cos(M_PI_2 + d_pred_pitch);
            double c = distance * distance;
            double d_c = d_distance * d_distance;
            if ((b * b - 4 * a * c) < 0 || (d_b * d_b - 4 * d_a * d_c) < 0) {
                //Log::Error("the speed incorrect !");
            }
            double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
            double d_t_2 = (-sqrt(d_b * d_b - 4 * d_a * d_c) - d_b) / (2 * d_a);
            double height = 0.5 * gravity * t_2;
            double d_height = 0.5 * gravity * d_t_2;

            shoot_pw = {pw(0, 0), pw(1, 0), pw(2, 0) - height};
            Eigen::Vector3d d_s_pw{d_pw(0, 0), d_pw(1, 0), d_pw(2, 0) - d_height};

            Eigen::Vector3d shoot_pc = coordinate.PwToPc(shoot_pw);
            Eigen::Vector3d d_shoot_pc = coordinate.PwToPc(d_shoot_pw);

            double yaw = atan(shoot_pc(0, 0) / shoot_pc(2, 0)) / M_PI * 180.;
            double d_yaw = atan(d_shoot_pc(0, 0) / d_shoot_pc(2, 0)) / M_PI * 180.;
            double pitch = atan(shoot_pc(1, 0) / shoot_pc(2, 0)) / M_PI * 180.;
            double d_pitch = atan(d_shoot_pc(1, 0) / d_shoot_pc(2, 0)) / M_PI * 180.;

            auto&& dx = (yaw) / 5. + px;
            auto&& dy = (pitch) / 10. + py;
            auto&& d_dx = (d_yaw - yaw) / 0.001 / 180. * M_PI * 2;
            auto&& d_dy = (d_pitch - pitch) / 0.001 / 180. * M_PI * 2;
            return Eigen::Vector4d(dx, dy, d_dx, d_dy);
        }

        /**
             * @brief 项目里原本的能量机关解算方法
             * @param pw 预测后的世界坐标系
             * @param coordinate 传入已经设置好的coordinate对象
             * @param v 弹丸速度
             * @return Eigen::Vector2d(dx,dy) -----dx为解算后的yaw，dy为解算后的pitch
            */
        Eigen::Vector2d CalOffSet(const Eigen::Vector3d& pw, Coordinate& coordinate, const float& v) {
            orin_pw = pw;
            velocity = v;
            double pred_pitch = atan2(pw(2, 0), pw.topRows<2>().norm());
            double distance = pw.norm();
            double a = gravity * gravity * 0.25;
            double b = -(double)velocity * velocity - distance * gravity * cos(M_PI_2 + pred_pitch);
            double c = distance * distance;
            if ((b * b - 4 * a * c) < 0) {
                Log::Error("the speed incorrect !");
            }
            double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
            // double fly_time = sqrt(t_2);
            double height = 0.5 * gravity * t_2;
            shoot_pw = {pw(0, 0), pw(1, 0), pw(2, 0) - height};      // 抬枪之后的世界坐标
            Eigen::Vector3d shoot_pc = coordinate.PwToPc(shoot_pw);  // 抬枪之后的相机坐标系下坐标
            double yaw = atan(shoot_pc(0, 0) / shoot_pc(2, 0)) / M_PI * 180.;
            double pitch = atan(shoot_pc(1, 0) / shoot_pc(2, 0)) / M_PI * 180.;

            auto&& dx = (yaw);
            auto&& dy = (pitch);

            return Eigen::Vector2d(dx, dy);
        }

        /**
             * @brief 绘制出完成提前量解算后需要瞄准的点
            */
        void Draw(cv::Mat& img, Coordinate& coordinate) {
            // Eigen::Vector3d shoot_pc = coordinate.PwToPc(shoot_pw);
            // auto&& shoot_pu = coordinate.PcToPu(shoot_pc);

            orin_pc = coordinate.PwToPc(orin_pw);
            orin_pu = coordinate.PcToPu(orin_pc);

            //Log::Info("shoot_pu = {}", shoot_pu);
            cv::circle(img, {int(shoot_pu(0, 0)), int(shoot_pu(1, 0))}, 5, Colors::Yellow, 3);  //抬枪之后的点
            // cv::circle(img, {int(orin_pu(0, 0)), int(orin_pu(1, 0))}, 5, Colors::Red, 3);       // 不抬枪的点
            cv::circle(img, cv::Point2f(640, 512), 2, Colors::Blue, 3);  // 图像中心点
        }

        /**
             *@brief 设置手动补偿量,px,py的物理含义为相机到摩擦轮出射处的偏移量
            */
        void SetHandOffSet(const double& x, const double& y) {
            px = x;
            py = y;
        }

    private:
        double gravity;     //重力系数
        double kofsmall;    //小弹丸风阻系数
        double koflarge;    //大弹丸风阻系数
        double koflight;    //荧光弹丸风阻系数
        int max_iter;       //龙格库塔法最大迭代次数
        double velocity;    //弹丸速度
        int R_K_iter;       //龙格库塔法求解落点的迭代次数
        double stop_error;  //停止迭代的最小误差(单位m)
        double k;           //风阻系数
        double px;          //yaw轴补偿
        double py;          //pitch轴补偿
    };
};                          // namespace phoenix