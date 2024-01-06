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

#include "auto_aim_interfaces/msg/rune_target.hpp"
#include "rune_tracker/tracker.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

#include "auto_aim_interfaces/msg/rune.hpp"
#include "pnp_solver.hpp"
#include "point.hpp"                 //添加Angle方法
#include "ring_buffer_statistic.hpp" //添加RingBufferStatistic模版类
#include "tracker.hpp"
#include "ukf_plus.h"             //添加UKF_PLUS类
#include <opencv2/core/types.hpp> //提供Point Point2d/2f
//  Ceres-solver
#include <ceres/ceres.h>

namespace rune {
// 使用tf2_ros::MessageFilter对自动瞄准接口的Armors消息进行过滤
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Rune>;

class RuneTrackerNode: public rclcpp::Node {
public:
    explicit RuneTrackerNode(const rclcpp::NodeOptions& option);

private:
    // 处理Rune消息的回调函数
    void RunesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr);

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
        CURVE_FITTING_COST(double x, double y):
            _x(x),
            _y(y) {}

        template<typename T>
        bool operator()(const T* const param, T* residual) const {
            residual[0] = T(_y) - (param[0] * sin(param[1] * _x + param[2]) + param[3]);
            return true;
        }
        const double _x, _y;
    };

    struct rune_tracker { //记录符叶的状态，主要用于计算当前参数的拟合误差
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
    } tracker;             //用于跟踪上一次的拟合情况

    bool SetRotate(const RotationDirection& rotation_direction) {
        static int cnt = 0;
        if (this->rotation_direction == rotation_direction) {
            cnt = 0;
            return false;
        }
        if (++cnt < 10) {
            return false;
        }
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
        this->motion_state = motion_state;
        Reset();
        return true;
    }
    //用于计算小符角速度
    void CalSmallRune();

    //判断顺时针还是逆时针
    bool Judge();

    bool FittingBig();

    bool Fitting();
    //ceres求解器求解 获得大符角速度参数 A omega phi b 并且验证预测参数是否正确
    bool CeresProcess();

    double integral(double w, std::vector<double> params, double t_s, double pred_time);

    void Reset() {
        speed.Clear();
        speeds.Clear();
        angles.Clear();
        tracker_->ukf->Clear();
        cere_param_list.clear();
    }
    //当delta_angle太大时，认为ceres拟合数据不准确，需要重新拟合
    void Refitting();
    //初始化txt的读取，用于记录原始角速度曲线和卡尔曼滤波曲线
    void InitRecord();

    //数据处理，判断角速度是否正常，正常则记录数据并且丢入ukf，传入的参数为符叶角度
    void DataProcess();

    // 发布标记点函数
    void PublishMarkers(const auto_aim_interfaces::msg::RuneTarget& target_msg);

    // 相机信息订阅者
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    // 相机中心
    cv::Point2f cam_center_;
    // 相机信息
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    // PnP 解算器
    std::unique_ptr<PnPSolver> pnp_solver_;

    double delay;      //理论延迟和追踪延迟之和
    double chasedelay; //追踪延迟 从launch参数给定
    double bullet_speed;
    std::shared_ptr<rclcpp::ParameterEventHandler> chasedelay_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> chasedelay_cb_handle_;

    // rclcpp::Time delay;//理论延迟和追踪延迟之和
    double leaf_angle, leaf_angle_last, leaf_angle_diff; //符叶角度 上一帧符叶角度 符叶角度差
    double leaf_angular_velocity;                        //符叶角速度
    double rotate_angle;                                 //预测符叶旋转角度
    cv::Point2f leaf_dir;                                //这一帧符叶向量

    double cere_rotated_angle;
    rclcpp::Time t_zero;                           //时间起点
    RingBufferStatistic<double, 1 << 3> angles {}; //符叶角度
    RingBufferStatistic<double, 1 << 3> speeds {}; //角速度
    Statistic<double> radius;                      //符叶半径
    Statistic<double> speed;                       //符叶角速度

    std::unique_ptr<Tracker> tracker_; //封装了ukf滤波器

    bool finish_fitting; //完成拟合的标志

    //count_cant_use用于不可用数据的计数，当数据突变时候，则过滤一定数目的数据之后丢入ukf
    int count_cere, count_cant_use, filter_astring_threshold;
    std::deque<CereParam> cere_param_list; //时域拟合的数据队列
    double a_omega_phi_b[4];               //拟合的参数
    ceres::Solver::Options options;        //解决方案的配置
    ceres::Solver::Summary summary;        //拟合的信息

    double pred_angle; //预测角度

    auto_aim_interfaces::msg::Rune::SharedPtr data;      //当前帧的数据
    auto_aim_interfaces::msg::Rune::SharedPtr data_last; //上一帧的数据

    std::ofstream omega_file;
    std::ofstream omega_time;
    std::ofstream origin_omega_file;
    std::ofstream origin_omega_time;

    //使用tf2_ros::MessageFilter对自动瞄准接口的Rune消息进行过滤
    message_filters::Subscriber<auto_aim_interfaces::msg::Rune> runes_sub_;
    std::string target_frame_;                                 // TODO: ???
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;              // tf2 缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_; // tf2 监听器
    std::shared_ptr<tf2_filter> tf2_filter_;

    rclcpp::Publisher<auto_aim_interfaces::msg::RuneTarget>::SharedPtr
        target_pub;                                  //向shooter节点发送数据
    auto_aim_interfaces::msg::RuneTarget runes_msg_; //自定义的神符信息

    // 可视化标记发布器
    // visualization_msgs::msg::Marker position_marker_;
    // visualization_msgs::msg::Marker linear_v_marker_;
    // visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker armor_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

} // namespace rune

#endif // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
