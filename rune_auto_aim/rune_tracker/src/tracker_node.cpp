// Copyright 2023 wangchi
#include "rune_tracker/tracker_node.hpp"
#include "rune_tracker/coordinate.h"

// STD
#include <memory>
#include <opencv2/core/types.hpp>
#include <vector>

namespace rune {
// RuneTrackerNode类的构造函数
RuneTrackerNode::RuneTrackerNode(const rclcpp::NodeOptions& option): Node("rune_tracker", option) {
    // 打印信息，表示节点已启动
    RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");
    coordinate = std::make_shared<Coordinate>();
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //选择最小二乘的拟合模式
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    finish_fitting = false;
    tracker.pred_time = 0;
    tracker.pred_angle = 0;
    tracker.angle = 0; //2023.9.7号做了修改 加上了tracker.angle的初始化
    options.minimizer_progress_to_stdout = false; //选择不打印拟合信息
    options.num_threads = 4; //使用4个线程进行拟合
    // horizon = 8;
    a_omega_phi_b[0] = RUNE_ROTATE_A_MIN;
    a_omega_phi_b[1] = RUNE_ROTATE_O_MIN;
    a_omega_phi_b[2] = 0;
    a_omega_phi_b[3] = 0;
    count_cere = 0;

    this->declare_parameter("chasedelay", 0); //设置chasedelay的默认值
    this->get_parameter("chasedelay", this->chasedelay); //参数更新
    tracker_ = std::make_unique<Tracker>(); //tracker中的ukf滤波器初始化

    // 重置追踪器服务
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // tf2相关
    // tf2 buffer & listener 相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // 订阅器和过滤器
    runes_sub_.subscribe(this, "/detector/runes", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    tf2_filter_ = std::make_shared<tf2_filter>(
        runes_sub_, // message_filters subscriber
        *tf2_buffer_, // tf2 buffer
        target_frame_, // frame this filter should attempt to transform to
        10, // size of the tf2 cache
        this->get_node_logging_interface(), // node logging interface
        this->get_node_clock_interface(), // node clock interface
        std::chrono::duration<int>(1) // timeout
    );
    // 注册回调函数
    tf2_filter_->registerCallback(&RuneTrackerNode::RunesCallback, this);

    target_pub = this->create_publisher<auto_aim_interfaces::msg::RuneTarget>(
        "/RuneTracker2Shooter",
        rclcpp::SensorDataQoS()
    );
    omega_file.open("./record/omega.txt"); //用于记录曲线
    if (!omega_file.is_open()) {
        while (true) {
            std::cout << "cannot open the omega.txt" << std::endl;
        }
    }
    omega_time.open("./record/omegatime.txt");
    if (!omega_time.is_open()) {
        while (true) {
            std::cout << "cannot open the omegatime.txt" << std::endl;
        }
    }

    //   // 测量发布器（用于调试）
    //   info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

    //   // 发布器
    //   target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    //     "/tracker/target", rclcpp::SensorDataQoS());

    //   // 可视化标记发布器
    //   // 参见 http://wiki.ros.org/rviz/DisplayTypes/Marker
    //   position_marker_.ns = "position";
    //   position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    //   position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    //   position_marker_.color.a = 1.0;
    //   position_marker_.color.g = 1.0;
    //   linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    //   linear_v_marker_.ns = "linear_v";
    //   linear_v_marker_.scale.x = 0.03;
    //   linear_v_marker_.scale.y = 0.05;
    //   linear_v_marker_.color.a = 1.0;
    //   linear_v_marker_.color.r = 1.0;
    //   linear_v_marker_.color.g = 1.0;
    //   angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    //   angular_v_marker_.ns = "angular_v";
    //   angular_v_marker_.scale.x = 0.03;
    //   angular_v_marker_.scale.y = 0.05;
    //   angular_v_marker_.color.a = 1.0;
    //   angular_v_marker_.color.b = 1.0;
    //   angular_v_marker_.color.g = 1.0;
    //   armor_marker_.ns = "armors";
    //   armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    //   armor_marker_.scale.x = 0.03;
    //   armor_marker_.scale.z = 0.125;
    //   armor_marker_.color.a = 1.0;
    //   armor_marker_.color.r = 1.0;
    //   marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

bool RuneTrackerNode::Judge() {
    constexpr double delta = 1e-2;
    if (rotation_direction == RotationDirection::Anticlockwise ? leaf_angle_diff < delta
                                                               : leaf_angle_diff < -delta)
    {
        if (SetRotate(RotationDirection::Anticlockwise)) {
            return false;
        }
    } else if (rotation_direction == RotationDirection::Clockwise ? leaf_angle_diff > -delta : leaf_angle_diff > delta)
    {
        if (SetRotate(RotationDirection::Clockwise)) {
            return false;
        }
    } else {
        if (SetRotate(RotationDirection::Static)) {
            return false;
        }
    }
    return false;
}

bool RuneTrackerNode::FittingBig() {
    if (motion_state != MotionState::Big) {
        return false;
    }

    if (data->find == false) { //丢识别

        // before_omega_file<< fabs(leaf_angle_diff) / (data->sensor->timestamp -last->sensor->timestamp)<<std::endl;
        // before_omega_time<< (data->sensor->timestamp - t_zero).GetSeconds()<<std::endl;
        //用旧数据预测
        // if (cere_param_list.size() < 150 && cere_param_list.size() != 0) {
        //     auto temp_omega = cere_param_list.back();
        //     cere_param_list.push_back(CereParam {
        //         temp_omega.omega,
        //         (rclcpp::Time(data->header.stamp) - t_zero).seconds() - 0.2 });
        //     // omega_file << temp_omega.omega << std::endl;
        //     // omega_time << (data->sensor->timestamp - t_zero).GetSeconds()-0.2<< std::endl;

        // } else if (cere_param_list.size() == 150) {
        //     cere_param_list.pop_front();
        //     auto temp_omega = cere_param_list.back();
        //     cere_param_list.push_back(CereParam {
        //         temp_omega.omega,
        //         (rclcpp::Time(data->header.stamp) - t_zero).seconds() - 0.2 });
        //     // omega_file << temp_omega.omega << std::endl;
        //     // omega_time << (data->sensor->timestamp - t_zero).GetSeconds()-0.2<< std::endl;
        // }
        return false;
    }

    if (cere_param_list.empty()) { //数据队列为空时，初始化
        tracker.timestamp = data->header.stamp; //记录时间戳
        t_zero = data->header.stamp; //时间起点
        cere_rotated_angle = leaf_angle; //记录第一帧符叶的角度
        tracker.pred_time = 0;
        tracker.angle = cere_rotated_angle;
    }

    if ((rclcpp::Time(data->header.stamp) - t_zero).seconds() > 30) { //符的数据过期

        Reset();
        return false;
    }

    if (abs(leaf_angle - leaf_angle_last) > 0.4) {
        //上一帧与这一帧的角度差值超过0.4，则判断为可激活的符叶已转换
        cere_rotated_angle = leaf_angle - leaf_angle_last + cere_rotated_angle; // 变换符叶初始角度
        if (cere_rotated_angle > M_PI)
            cere_rotated_angle -= 2 * M_PI;
        else if (cere_rotated_angle < -M_PI)
            cere_rotated_angle += 2 * M_PI;
        RCLCPP_INFO(this->get_logger(), "rune_leaf change!");

        // return false;
    }
    if (cere_param_list.size() < 100) { //数据队列设为100个，数据队列未满
        auto&& theta = leaf_angle; //观测到这一帧符叶的角度

        // before_omega_file<< fabs(leaf_angle_diff) / (data->header.stamp - data_last->header.stamp)<<std::endl;
        // before_omega_time<<(data->sensor->timestamp - t_zero).GetSeconds()<<std::endl;

        MeasurementPackage package = MeasurementPackage(
            (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
            MeasurementPackage::SensorType::LASER,
            Eigen::Vector2d { RUNE_ARMOR_TO_SYMBOL * cos(theta - cere_rotated_angle),
                              RUNE_ARMOR_TO_SYMBOL * sin(theta - cere_rotated_angle) }
        );
        //将传感器的坐标数据丢入UKF
        //ukf输入坐标，输出估计的状态向量x_为[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        tracker_->ukf->ProcessMeasurement(package); //估计当前真实的状态
        double&& omega =
            1.0 * abs(tracker_->ukf->x_(2)) / RUNE_ARMOR_TO_SYMBOL; //从状态估计器中取出估计的omega

        cere_param_list.push_back(CereParam {
            .omega = omega,
            .time = (rclcpp::Time(data->header.stamp) - t_zero).seconds() - 0.2 });

        omega_file << omega << std::endl;
        // omega_time << (data->sensor->timestamp - t_zero).GetSeconds()-0.2 << std::endl;//-0.2是为了补偿相位差
        omega_time << (rclcpp::Time(data->header.stamp) - t_zero).seconds() << std::endl;
        runes_msg_.can_shoot = false;

        return false;
    } else if (cere_param_list.size() == 100) { //队列数据已满

        cere_param_list.pop_front(); //队列头数据弹出

        auto&& theta = leaf_angle; //当前符叶的角度(弧度制)

        // before_omega_file<< fabs(leaf_angle_diff) / (data->sensor->timestamp-last->sensor->timestamp)<<std::endl;
        // before_omega_time<<(data->sensor->timestamp - t_zero).GetSeconds()<<std::endl;

        MeasurementPackage package = MeasurementPackage(
            (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
            MeasurementPackage::SensorType::LASER,
            Eigen::Vector2d { RUNE_ARMOR_TO_SYMBOL * cos(theta - cere_rotated_angle),
                              RUNE_ARMOR_TO_SYMBOL * sin(theta - cere_rotated_angle) }
        );
        //ukf
        tracker_->ukf->ProcessMeasurement(package);
        double&& omega = 1.0 * abs(tracker_->ukf->x_(2)) / RUNE_ARMOR_TO_SYMBOL;

        cere_param_list.push_back(CereParam {
            .omega = omega,
            .time = (rclcpp::Time(data->header.stamp) - t_zero).seconds() - 0.2 });
        omega_file << omega << std::endl;
        // omega_time << (data->sensor->timestamp - t_zero).GetSeconds()-0.2 << std::endl;//-0.3是为了补偿相位差
        omega_time << (rclcpp::Time(data->header.stamp) - t_zero).seconds() << std::endl;

        if (finish_fitting) {
            RCLCPP_INFO(this->get_logger(), "predict correct");
        } else {
            RCLCPP_INFO(this->get_logger(), "predict inaccuracy,restart fittting");
        }
        if ((rclcpp::Time(data->header.stamp) - rclcpp::Time(tracker.timestamp)).seconds()
            >= tracker.pred_time)
        { //开始判断拟合参数的误差
            RCLCPP_INFO(this->get_logger(), "varify predict");

            // 计算误差
            double delta_angle = 0;
            if (this->rotation_direction == RotationDirection::Anticlockwise
                && tracker.pred_angle > 0)
            {
                tracker.pred_angle *= -1;
            }
            delta_angle = fabs(leaf_angle - (tracker.angle + tracker.pred_angle));
            if (delta_angle > M_PI) {
                delta_angle = fabs(2 * M_PI - delta_angle);
            }
            std::cout << "delta_angle is " << delta_angle << std::endl;
            // error_file << delta_angle << std::endl;
            // error_time << (data->sensor->timestamp - t_zero).GetSeconds() << std::endl;

            if (delta_angle < 0.15) { //误差小于0.1,则认为拟合良好
                // pred_angle = integral(a_omega_phi_b[1], std::vector<double>{a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3]}, (data->sensor->timestamp.Now() - t_zero).GetSeconds(), (data->sensor->timestamp.Now() - data->sensor->timestamp).GetSeconds() + delay);
                pred_angle = integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    delay + (rclcpp::Clock().now() - data->header.stamp).seconds()
                );
                runes_msg_.can_shoot = true; //可以发射
                count_cere = 0; //将count_cere置为0，非连续5次拟合不良
                finish_fitting = true;
            } else { //误差大于0.15,则认为拟合不良
                if (count_cere < 5) {
                    count_cere++;
                    // tracker.pred_angle = integral(a_omega_phi_b[1], std::vector<double>{a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3]}, (data->sensor->timestamp - t_zero).GetSeconds(), delay);
                    tracker.pred_angle = integral(
                        a_omega_phi_b[1],
                        std::vector<double> { a_omega_phi_b[0],
                                              a_omega_phi_b[2],
                                              a_omega_phi_b[3] },
                        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                        delay + (rclcpp::Clock().now() - data->header.stamp).seconds()
                    );
                    tracker.timestamp = data->header.stamp;
                    // tracker.pred_time = delay;
                    tracker.pred_time =
                        delay + (rclcpp::Clock().now() - data->header.stamp).seconds();
                    tracker.angle = leaf_angle; //记录当前角度 用于后续的拟合检测

                    // pred_angle = integral(a_omega_phi_b[1], std::vector<double>{a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3]}, (data->sensor->timestamp.Now() - t_zero).GetSeconds(), delay);
                    pred_angle = integral(
                        a_omega_phi_b[1],
                        std::vector<double> { a_omega_phi_b[0],
                                              a_omega_phi_b[2],
                                              a_omega_phi_b[3] },
                        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                        delay + (rclcpp::Clock().now() - data->header.stamp).seconds()
                    );
                    return false;
                }
                //runes_msg_.can_shoot = false;  //连续五次误差超过阈值，认为不能发射
                finish_fitting = false; //连续五次误差超过0.1，则认为需要重新拟合
                count_cere = 0;
                ceres::Problem problem;
                for (unsigned long i = 0; i < cere_param_list.size(); i++) { //将数据添加入问题
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                            new CURVE_FITTING_COST(
                                cere_param_list[i].time,
                                cere_param_list[i].omega
                            )
                        ),
                        new ceres::CauchyLoss(0.1),
                        a_omega_phi_b
                    );
                }

                problem.SetParameterLowerBound(a_omega_phi_b, 0, 0.78); //设置参数上下限
                problem.SetParameterUpperBound(a_omega_phi_b, 0, 1.045);
                problem.SetParameterLowerBound(a_omega_phi_b, 1, 1.884); // 数据是官方的
                problem.SetParameterUpperBound(a_omega_phi_b, 1, 2);
                // problem.SetParameterLowerBound(a_omega_phi_b, 0, 0.5); //实验室符参数
                // problem.SetParameterUpperBound(a_omega_phi_b, 0, 0.9);
                // problem.SetParameterLowerBound(a_omega_phi_b, 1,1.6);
                // problem.SetParameterUpperBound(a_omega_phi_b, 1, 2.0);

                problem.SetParameterLowerBound(a_omega_phi_b, 2, -1 * M_PI);
                problem.SetParameterUpperBound(a_omega_phi_b, 2, 1 * M_PI);
                problem.SetParameterLowerBound(a_omega_phi_b, 3, 1.045);
                problem.SetParameterUpperBound(a_omega_phi_b, 3, 1.310);
                // problem.SetParameterLowerBound(a_omega_phi_b, 3, 1.59);
                // problem.SetParameterUpperBound(a_omega_phi_b, 3, 1.19);

                ceres::Solve(options, &problem, &summary); //开始拟合(解决问题)
                //Log::Info("fitting params is a : {}   omega : {}    phi : {},b : {}", a_omega_phi_b[0], a_omega_phi_b[1], a_omega_phi_b[2], a_omega_phi_b[3]);
                // tracker.pred_time = delay;
                tracker.pred_time = delay
                    + (rclcpp::Clock().now() - rclcpp::Time(data->header.stamp))
                          .seconds(); //更新下一次重新判断拟合的时间
                // tracker.pred_angle = integral(a_omega_phi_b[1], std::vector<double>{a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3]}, (data->sensor->timestamp - t_zero).GetSeconds(), tracker.pred_time);  //当前参数预测旋转的角度
                tracker.pred_angle = integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    delay + (rclcpp::Clock().now() - data->header.stamp).seconds()
                );

                // pred_angle = integral(a_omega_phi_b[1], std::vector<double>{a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3]}, (data->sensor->timestamp.Now() - t_zero).GetSeconds(), delay + (data->sensor->timestamp.Now() - data->sensor->timestamp).GetSeconds());  //当前参数预测旋转的角度
                pred_angle = integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    (delay + (rclcpp::Clock().now() - data->header.stamp).seconds())
                );
                tracker.angle = leaf_angle;
                tracker.timestamp = data->header.stamp;
                runes_msg_.can_shoot = true;
                return true;
            }
        } //还没有到达预测的时间
        else
        {
            // pred_angle = integral(a_omega_phi_b[1], std::vector<double>{a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3]}, (data->sensor->timestamp.Now() - t_zero).GetSeconds(), (data->sensor->timestamp.Now() - data->sensor->timestamp).GetSeconds() + delay);
            pred_angle = integral(
                a_omega_phi_b[1],
                std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2], a_omega_phi_b[3] },
                (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                (delay + (rclcpp::Clock().now() - data->header.stamp).seconds())
            );
            runes_msg_.can_shoot = false;
            return true;
        }
    }
    return true;
}

bool RuneTrackerNode::Fitting() {
    switch (motion_state) {
        case MotionState::Static: {
            rotate_angle = 0;
        } break;
        case MotionState::Small: {
            switch (rotation_direction) {
                case RotationDirection::Clockwise: {
                    rotate_angle = speed.Mean() * delay;
                    runes_msg_.can_shoot = true;
                } break;
                case RotationDirection::Anticlockwise: {
                    rotate_angle = -speed.Mean() * delay;
                    runes_msg_.can_shoot = true;
                } break;
                default: {
                    runes_msg_.can_shoot = false;
                    return false;
                }
            }
        } break;
        case MotionState::Big: {
            switch (rotation_direction) {
                case RotationDirection::Clockwise: {
                    rotate_angle = pred_angle;
                } break;
                case RotationDirection::Anticlockwise: {
                    rotate_angle = -pred_angle;
                } break;
                default: {
                    return false;
                } break;
            }
        } break;
        default: {
            return false;
        }
    }
    return true;
}

double
RuneTrackerNode::integral(double w, std::vector<double> params, double t_s, double pred_time) {
    // std::cout << "in integral" << std::endl;
    double a = params[0];
    double phi = params[1];
    double t_e = t_s + pred_time;
    double theta1 = -a / w * cos(w * t_s + phi) + (params[2]) * t_s;
    double theta2 = -a / w * cos(w * t_e + phi) + (params[2]) * t_e;
    return theta2 - theta1;
} //积分 用于预测下个时刻的位置

// runeCallback函数实现 接收rune_detector发布的rune消息
void RuneTrackerNode::RunesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr) {
    RCLCPP_INFO(this->get_logger(), "receive rune message");
    data = rune_ptr;
    if (rune_ptr->find) //如果detector识别到了符装甲板
    {
        auto&& theory_delay = rune_ptr->pose_c.position.z / 27;
        delay = theory_delay + chasedelay;
        runes_msg_.delay = delay;
        runes_msg_.header = rune_ptr->header; //时间戳赋值
        RCLCPP_INFO(this->get_logger(), "delay: %f", delay);

        // if (data->motion == 0) {
        //     SetState(MotionState::Static);
        // } else if (data->motion == 1) {
        //     SetState(MotionState::Small);
        // } else if (data->motion == 2) {
        //     SetState(MotionState::Big);
        // }//从下位机来的数据，判断是静止还是小符还是大符

        SetState(MotionState::Big); //缺省设置为大符

        cv::Point2f tmp_dir(rune_ptr->leaf_dir.x, rune_ptr->leaf_dir.y); //符四个点中心到R标

        this->leaf_dir = tmp_dir; //现在这一帧符叶向量

        leaf_angle = Angle(leaf_dir); //返回弧度制的角度
        if (angles.Any()) {
            leaf_angle_diff = Revise(leaf_angle - leaf_angle_last, -36_deg, 36_deg); //修正范围
            auto&& leaf_angle_diff_abs = std::abs(leaf_angle_diff);
            angles.PushForcibly(angles[-1] + leaf_angle_diff_abs);
            //下面求解角速度
            speed.Push(
                leaf_angle_diff_abs
                / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp))
                      .seconds()
            );
            speeds.PushForcibly(speed.Value());
        } else {
            angles.PushForcibly(leaf_angle);
        }
        radius.Push(cv::norm(leaf_dir)); //计算半径
    }
    Judge(); //判断顺时针还是逆时针
    FittingBig(); //拟合大符
    Fitting(); //拟合小符

    data_last = data; //记录上一帧的数据
    leaf_angle_last = leaf_angle; //记录上一帧的角度
    if (data->find) {
        std::vector<cv::Point2d> rotate_armors;
        cv::Point2d symbol(data->symbol.x, data->symbol.y); //R标
        for (int i = 0; i < 4; i++) {
            rotate_armors.emplace_back(cv::Point2d(data->rune_points[i].x, data->rune_points[i].y));
        }
        for (auto&& vertex: rotate_armors) { //将关键点以圆心旋转rotate_angle
            vertex = Rotate(vertex, symbol, rotate_angle);
        }

        auto&& pc = coordinate->PnpGetPc(ArmorType::Rune, rotate_armors); //相机坐标系下的装甲板坐标
        auto&& pw =
            coordinate->RunePcToPw(pc); //将相机坐标系下装甲板坐标转换成世界坐标系下的装甲板坐标
        runes_msg_.pw.position.x = pw[0];
        runes_msg_.pw.position.y = pw[1];
        runes_msg_.pw.position.z = pw[2];
        target_pub->publish(runes_msg_);
    }
}

// void RuneTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
// {
//   position_marker_.header = target_msg.header;
//   linear_v_marker_.header = target_msg.header;
//   angular_v_marker_.header = target_msg.header;
//   armor_marker_.header = target_msg.header;

//   visualization_msgs::msg::MarkerArray marker_array;
//   if (target_msg.tracking) {
//     double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
//     double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
//     double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
//     double dz = target_msg.dz;

//     position_marker_.action = visualization_msgs::msg::Marker::ADD;
//     position_marker_.pose.position.x = xc;
//     position_marker_.pose.position.y = yc;
//     position_marker_.pose.position.z = za + dz / 2;

//     linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
//     linear_v_marker_.points.clear();
//     linear_v_marker_.points.emplace_back(position_marker_.pose.position);
//     geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
//     arrow_end.x += vx;
//     arrow_end.y += vy;
//     arrow_end.z += vz;
//     linear_v_marker_.points.emplace_back(arrow_end);

//     angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
//     angular_v_marker_.points.clear();
//     angular_v_marker_.points.emplace_back(position_marker_.pose.position);
//     arrow_end = position_marker_.pose.position;
//     arrow_end.z += target_msg.v_yaw / M_PI;
//     angular_v_marker_.points.emplace_back(arrow_end);

//     armor_marker_.action = visualization_msgs::msg::Marker::ADD;
//     armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
//     bool is_current_pair = true;
//     size_t a_n = target_msg.armors_num;
//     geometry_msgs::msg::Point p_a;
//     double r = 0;
//     for (size_t i = 0; i < a_n; i++) {
//       double tmp_yaw = yaw + i * (2 * M_PI / a_n);
//       // 只有4个装甲板有2个半径和高度
//       if (a_n == 4) {
//         r = is_current_pair ? r1 : r2;
//         p_a.z = za + (is_current_pair ? 0 : dz);
//         is_current_pair = !is_current_pair;
//       } else {
//         r = r1;
//         p_a.z = za;
//       }
//       p_a.x = xc - r * cos(tmp_yaw);
//       p_a.y = yc - r * sin(tmp_yaw);

//       armor_marker_.id = i;
//       armor_marker_.pose.position = p_a;
//       tf2::Quaternion q;
//       q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
//       armor_marker_.pose.orientation = tf2::toMsg(q);
//       marker_array.markers.emplace_back(armor_marker_);
//     }
//   } else {
//     position_marker_.action = visualization_msgs::msg::Marker::DELETE;
//     linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
//     angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

//     armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
//     marker_array.markers.emplace_back(armor_marker_);
//   }

//   marker_array.markers.emplace_back(position_marker_);
//   marker_array.markers.emplace_back(linear_v_marker_);
//   marker_array.markers.emplace_back(angular_v_marker_);
//   marker_pub_->publish(marker_array);
// }

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// 用class_loader注册组件。
// 这充当一种入口点，允许在将其库加载到运行中的进程时发现组件。
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneTrackerNode)
