
#pragma once

// ROS
#include "auto_aim_interfaces/msg/debug_rune.hpp"
#include "auto_aim_interfaces/msg/rune.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include <rclcpp/rclcpp.hpp>

#include "point.hpp" //添加Angle方法
#include "statistic.hpp"
#include "ukf_plus.h"
#include <opencv2/core/types.hpp> //提供Point Point2d/2f
#include <rune_detector/sizes.hpp>

//  Ceres-solver
#include <ceres/ceres.h>

namespace rune {
// 装甲板追踪器类
class Tracker {
public:
    Tracker(rclcpp::Node* node, double&& std_a_, double&& std_yawdd_, int& filter_astring_threshold_);
    using Rune = auto_aim_interfaces::msg::Rune;

    //node调用tracker功能函数
    void Predict(const auto_aim_interfaces::msg::Rune::SharedPtr& data, auto_aim_interfaces::msg::Target& msg, auto_aim_interfaces::msg::DebugRune& debug_msg);

    //返回旋转预测角后的符叶坐标
    std::vector<cv::Point2d>& GetRotatedRune() {
        return rotate_armors;
    }

    //获取旋转角度(预测角)
    double GetRotateAngle() {
        return rotate_angle;
    }

    //返回ceres拟合参数a_omega_phi_b数组的指针
    double* GetFittingPara() {
        return a_omega_phi_b;
    }

private:
    enum class MotionState {
        UNKNOWN,
        STATIC,
        SMALL,
        BIG
    } motion_state_ = MotionState::UNKNOWN; //符叶运动状态 未知 静止 小符 大符 下位机传上来的数据

    enum class RotationDirection {
        UNKNOWN,
        STATIC,
        CLOCKWISE,
        ANTICLOCKWISE
    } rotation_direction_ = RotationDirection::UNKNOWN; //旋转方向 这个由程序自己判断

    struct CereParam {
        double omega;
        double time;
    };
    struct CURVE_FITTING_COST {
        CURVE_FITTING_COST(double x, double y):
            _x(x),
            _y(y) {}

        template<typename T>
        bool operator()(const T* const param, T* residual) const {
            residual[0] = T(_y) - (param[0] * ceres::sin(param[1] * _x + param[2]) + param[3]);
            return true;
        }
        const double _x, _y;
    };

    struct RuneTracker {
        //记录符叶的状态，主要用于计算当前参数的拟合误差
        double angle;
        rclcpp::Time timestamp;
        double angle_speed;
        cv::Point2f symbol;
        double a;
        double b;
        double phi;
        double omega;
        double pred_angle; //预测的角度
        double pred_time;
        cv::Point2d armor; //用于追踪上一次的符叶点
        //记录符叶的状态，主要用于计算当前参数的拟合误差
        inline void Record(double& pred_angle_, const rclcpp::Time& timestamp_, const double& pred_time_, double& angle_) {
            pred_angle = pred_angle_;
            timestamp = timestamp_;
            pred_time = pred_time_;
            angle = angle_;
        }
    } tracker; //用于跟踪上一次的拟合情况

    bool SetRotate(const RotationDirection& rotation_direction) {
        static int cnt = 0;
        if (this->rotation_direction_ == rotation_direction) {
            cnt = 0;
            return false;
        }
        if (++cnt < 10) {
            return false;
        }
        this->rotation_direction_ = rotation_direction;
        Reset();
        return true;
    }

    bool SetState(const MotionState& motion_state) {
        static int cnt = 0;
        if (this->motion_state_ == motion_state) {
            cnt = 0;
            return false;
        }
        if (++cnt < 10) {
            return false;
        }
        this->motion_state_ = motion_state;
        Reset();
        return true;
    }
    void Reset() {
        speed.Clear();
        ukf_->Clear();
        cere_param_list.clear();
    }

    //当delta_angle太大时，认为ceres拟合数据不准确，需要重新拟合
    void Refitting();

    void InitCeres(); //初始化ceres求解器

    //用于计算小符角速度
    void CalSmallRune(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::DebugRune& debug_msg);

    //判断顺时针还是逆时针
    bool Judge(auto_aim_interfaces::msg::DebugRune& debug_msg);

    //拟合大符参数并计算预测角度
    bool FittingBig(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg);

    //predict_angle计算
    bool Fitting(auto_aim_interfaces::msg::Target& runes_msg);

    //数据处理，判断角速度是否正常，正常则记录数据并且丢入ukf，传入的参数为符叶角度
    void DataProcess(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::DebugRune& debug_msg);

    //ceres求解器求解 获得大符角速度参数 A omega phi b 并且验证预测参数是否正确
    bool CeresProcess(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg);

    //积分,用于大符预测角度计算
    inline double Integral(double& w, const std::vector<double>& params, const double& t_s, const double& pred_time) {
        double a = params[0];
        double phi = params[1];
        double t_e = t_s + pred_time;
        double theta1 = -a / w * cos(w * t_s + phi) + (params[2]) * t_s;
        double theta2 = -a / w * cos(w * t_e + phi) + (params[2]) * t_e;
        return theta2 - theta1;
    };

    //theta为当前角度，cere_rotated_angle为基准角度
    double AngleRevise(double& theta, double& cere_rotated_angle) {
        if (cere_rotated_angle > 0 && theta < cere_rotated_angle - M_PI)
        {
            return 2 * M_PI + theta - cere_rotated_angle;
        } else if (cere_rotated_angle < 0 && theta > cere_rotated_angle + M_PI)
        {
            return -2 * M_PI + theta - cere_rotated_angle;
        } else
        {
            return theta - cere_rotated_angle;
        }
    }

    Filter* ukf_; // ukf滤波器
    double delay; //理论延迟和追踪延迟之和

    double leaf_angle, leaf_angle_last, leaf_angle_diff; //符叶角度 上一帧符叶角度 符叶角度差
    double leaf_angular_velocity;                        //符叶角速度
    double rotate_angle;                                 //预测符叶旋转角度
    std::vector<cv::Point2d> rotate_armors;              //旋转后的装甲板坐标

    double cere_rotated_angle;
    rclcpp::Time t_zero;     //时间起点
    Statistic<double> speed; //符叶角速度

    bool finish_fitting; //完成拟合的标志
    // count_cant_use用于不可用数据的计数，当数据突变时候，则过滤一定数目的数据之后丢入ukf
    int count_cere, count_cant_use, filter_astring_threshold;
    std::deque<CereParam> cere_param_list; //时域拟合的数据队列
    double a_omega_phi_b[4];               //拟合的参数
    double phase_offset;                   //相位差补偿,用于补偿观测到的角速度和滤波后角速度的相位差

    ceres::Solver::Options options; //解决方案的配置
    ceres::Solver::Summary summary; //拟合的信息

    double pred_angle;                                   //预测角度
    auto_aim_interfaces::msg::Rune::SharedPtr data_last; //上一帧的数据
    rclcpp::Node* node_;
};

} // namespace rune
