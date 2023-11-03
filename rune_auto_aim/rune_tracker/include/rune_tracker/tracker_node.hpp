// Copyright 2023 wangchi
#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <auto_aim_interfaces/msg/detail/rune_target__struct.hpp>
#include <message_filters/subscriber.h>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
// #include <eigen_conversions/eigen_msg.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "rune_tracker/tracker.hpp"
// #include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/rune_target.hpp"
// #include "auto_aim_interfaces/msg/tracker_info.hpp"

#include <opencv2/core/types.hpp> //提供Point Point2d/2f

#include "auto_aim_interfaces/msg/rune.hpp"
#include "point.hpp" //添加Angle方法
#include "ring_buffer_statistic.hpp" //添加RingBufferStatistic模版类
#include "sizes.hpp"
#include "statistic.hpp" //添加Statistic模版类
#include "timestamp.h" //添加Timestamp类
#include "tracker.hpp"
#include "ukf_plus.h" //添加UKF_PLUS类
//  Ceres-solver
#include <ceres/ceres.h>
// Coordinate
#include "coordinate.h"

namespace rune {
// 使用tf2_ros::MessageFilter对自动瞄准接口的Armors消息进行过滤
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Rune>;

class RuneTrackerNode: public rclcpp::Node {
public:
    explicit RuneTrackerNode(const rclcpp::NodeOptions& option);

private:
    // 处理Rune消息的回调函数
    void runesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr);

    enum class MotionState {
        Unknown,
        Static,
        Small,
        Big
    } motion_state = MotionState::Unknown; //符叶运动状态 未知 静止 小符 大符 下位机传上来的数据

    enum class RotationDirection {
        Unknown,
        Static,
        Clockwise,
        Anticlockwise
    } rotation_direction = RotationDirection::Unknown; //旋转方向 这个由程序自己判断

    struct CereParam {
        double omega;
        double time;
    };

    struct CURVE_FITTING_COST {
        CURVE_FITTING_COST(double x, double y): _x(x), _y(y) {}

        template<typename T>
        bool operator()(const T* const param, T* residual) const {
            residual[0] = T(_y) - (param[0] * sin(param[1] * _x + param[2]) + param[3]);
            return true;
        }
        const double _x, _y;
    };

    struct rune_tracker { //记录符叶的状态，主要用于计算当前参数的拟合误差
        double angle;
        // Timestamp timestamp;
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
    } tracker; //用于跟踪上一次的拟合情况

    bool SetRotate(const RotationDirection& rotation_direction) {
        static int cnt = 0;
        if (this->rotation_direction == rotation_direction) {
            cnt = 0;
            return false;
        }
        if (++cnt < 10) {
            return false;
        }
        // Log::Debug("RotationDirection Changed From {} to {}", this->rotation_direction, rotation_direction);
        this->rotation_direction = rotation_direction;
        Reset();
        return true;
    }

    bool SetState(const MotionState& motion_state) {
        static int cnt = 0;
        if (this->motion_state == motion_state) {
            cnt = 0;
            return false;
        }
        if (++cnt < 10) {
            return false;
        }
        // Log::Debug("MotionState Changed From {} to {}", this->motion_state, motion_state);
        this->motion_state = motion_state;
        Reset();
        return true;
    }

    bool Judge();

    bool FittingBig();

    bool Fitting();

    double integral(double w, std::vector<double> params, double t_s, double pred_time);

    void Reset() {
        speed.Clear();
        speeds.Clear();
        angles.Clear();

        // data_enough = false;
        // quality_guaranteed = false;
        // curve_done = false;

        tracker_->ukf->Clear();

        cere_param_list.clear();
        //Log::Debug("Rune::Reset()");
    }

    double delay; //理论延迟和追踪延迟之和
    double chasedelay; //追踪延迟

    // rclcpp::Time delay;//理论延迟和追踪延迟之和
    double leaf_angle, leaf_angle_last, leaf_angle_diff; //符叶角度 上一帧符叶角度 符叶角度差
    double rotate_angle; //预测符叶旋转角度
    cv::Point2f leaf_dir; //这一帧符叶向量

    double cere_rotated_angle;
    rclcpp::Time t_zero; //时间起点
    // std_msgs::msg::Header t_zero;//用于记录时间起点
    RingBufferStatistic<double, 1 << 3> angles {}; //符叶角度
    RingBufferStatistic<double, 1 << 3> speeds {}; //角速度
    Statistic<double> radius; //符叶半径
    Statistic<double> speed; //符叶角速度

    std::shared_ptr<Coordinate> coordinate; //坐标系转换类
    std::unique_ptr<Tracker> tracker_; //ukf滤波器

    bool finish_fitting; //完成拟合的标志

    int count_cere;
    std::deque<CereParam> cere_param_list; //时域拟合的数据队列
    double a_omega_phi_b[4]; //拟合的参数
    ceres::Solver::Options options; //解决方案的配置
    ceres::Solver::Summary summary; //拟合的信息
    Timestamp fitting_start, fitting_end; //记录拟合的用时

    double pred_angle; //预测角度

    auto_aim_interfaces::msg::Rune::SharedPtr data; //当前帧的数据
    auto_aim_interfaces::msg::Rune::SharedPtr data_last; //上一帧的数据

    std::ofstream omega_file;
    std::ofstream omega_time;
    std::ofstream origin_omega_file;
    std::ofstream origin_omega_time;
    std::ofstream error_file;
    std::ofstream error_time; //用于记录拟合的误差

    rclcpp::Publisher<auto_aim_interfaces::msg::RuneTarget>::SharedPtr
        target_pub; //向shooter节点发送数据
    auto_aim_interfaces::msg::RuneTarget runes_msg_; //自定义的神符信息

    //-----------------------------------------------------------------------------以下是装甲板部分的变量 与符无关
    // 发布标记点函数
    //   void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

    //   // 上次接收消息的时间
    //   rclcpp::Time last_time_;
    //   double dt_;

    //   // 使用tf2消息过滤器的订阅器
    //   std::string target_frame_;
    //   std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    //   std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    //   message_filters::Subscriber<auto_aim_interfaces::msg::Rune> armors_sub_;
    //   std::shared_ptr<tf2_filter> tf2_filter_;

    //   // 追踪器信息发布器
    //   rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

    //   // 发布器
    //   rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

    //   // 可视化标记发布器
    //   visualization_msgs::msg::Marker position_marker_;
    //   visualization_msgs::msg::Marker linear_v_marker_;
    //   visualization_msgs::msg::Marker angular_v_marker_;
    //   visualization_msgs::msg::Marker armor_marker_;
    //   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

} // namespace rune

#endif // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
