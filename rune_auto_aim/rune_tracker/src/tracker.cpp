#include "rune_tracker/tracker.hpp"
#define NEW_METHOD false
namespace rune {
Tracker::Tracker(rclcpp::Node* node, double& std_a_, double& std_yawdd_, int& filter_astring_threshold_):
    node_(node) {
    RCLCPP_INFO(node_->get_logger(), "Starting RuneTracker!");
    ukf_ = new UKF_PLUS(false, true, false, std_a_, std_yawdd_);
    this->filter_astring_threshold = filter_astring_threshold_;
    InitCeres(); //初始化ceres
}

void Tracker::Predict(const auto_aim_interfaces::msg::Rune::SharedPtr& data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    auto&& theory_delay = data->pose_c.position.z / data->speed;
    debug_msg.delay = runes_msg.delay = delay = theory_delay + data->chasedelay;
    runes_msg.header = data->header; //时间戳赋值
    phase_offset = data->phase_offset;
    if (data->motion == 0) {
        SetState(MotionState::STATIC);
        debug_msg.motion_state = "Static";
    } else if (data->motion == 1) {
        SetState(MotionState::SMALL);
        debug_msg.motion_state = "Small";
    } else if (data->motion == 2) {
        SetState(MotionState::BIG);
        debug_msg.motion_state = "Big";
    } //从下位机来的数据，判断是静止还是小符还是大符

    cv::Point2f tmp_dir(data->leaf_dir.x, data->leaf_dir.y); //符四个点中心到R标
    leaf_angle = Angle(std::move(tmp_dir));                  //返回弧度制的角度
    RCLCPP_DEBUG(node_->get_logger(), "leaf_angle %lf", leaf_angle);
    leaf_angle_diff = Revise(leaf_angle - leaf_angle_last, -36_deg, 36_deg); //修正后的角度差
    CalSmallRune(data, debug_msg);                                           //计算小符角速度
    Judge(debug_msg);                                                        //判断顺时针还是逆时针
    FittingBig(data, runes_msg, debug_msg);                                  //拟合大符
    Fitting(runes_msg);                                                      //计算预测角度
    data_last = data;                                                        //记录上一帧的数据
    leaf_angle_last = leaf_angle;                                            //记录上一帧的角度
    cv::Point2d symbol(data->symbol.x, data->symbol.y);                      //R标
    rotate_armors.clear();
    for (int i = 0; i < 4; i++) {
        rotate_armors.emplace_back(data->rune_points[i].x, data->rune_points[i].y);
    }
    for (auto&& vertex: rotate_armors) {
        //将关键点以圆心旋转rotate_angle 得到预测点
        vertex = Rotate(vertex, symbol, rotate_angle);
    }
}

void Tracker::InitCeres() {
    finish_fitting = false;
    tracker.pred_time = 0;
    tracker.pred_angle = 0;
    tracker.angle = 0;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //选择最小二乘的拟合模式
    options.minimizer_progress_to_stdout = false;              //选择不打印拟合信息
    options.num_threads = 4;                                   //使用4个线程进行拟合
    a_omega_phi_b[0] = RUNE_ROTATE_A_MIN;
    a_omega_phi_b[1] = RUNE_ROTATE_O_MIN;
    a_omega_phi_b[2] = 0;
    a_omega_phi_b[3] = 0;
    count_cere = 0;
}

void Tracker::CalSmallRune(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    if (motion_state_ != MotionState::SMALL) {
        return;
    }

    if (fabs(leaf_angle - leaf_angle_last) > 0.8) {
        //小符更换扇叶 则这一次的数据不做处理
        return;
    }
    auto&& leaf_angle_diff_abs = std::abs(leaf_angle_diff);
    //下面求解角速度
    speed.Push(
        leaf_angle_diff_abs
        / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp))
              .seconds()
    );
    debug_msg.small_rune_speed = speed.Mean();
}

bool Tracker::FittingBig(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    if (motion_state_ != MotionState::BIG) {
        return false;
    }
    if (cere_param_list.empty()) {              //数据队列为空时，初始化
        tracker.timestamp = data->header.stamp; //记录时间戳
        t_zero = data->header.stamp;            //时间起点
        cere_rotated_angle = leaf_angle;        //记录第一帧符叶的角度
        tracker.pred_time = 0;
        tracker.angle = cere_rotated_angle;
    }

    if ((rclcpp::Time(data->header.stamp) - t_zero).seconds() > 30.0) {
        //符的数据过期
        Reset();
        return false;
    }
#if !NEW_METHOD
    if (abs(leaf_angle - leaf_angle_last) > 0.4 && abs(leaf_angle - leaf_angle_last) < 5.2) {
        //上一帧与这一帧的角度差值超过阈值，则判断为可激活的符叶已转换
        cere_rotated_angle = leaf_angle - leaf_angle_last + cere_rotated_angle; // 变换符叶初始角度
        //角度变换后，需要矫正角度
        cere_rotated_angle = cere_rotated_angle > M_PI ? cere_rotated_angle - 2 * M_PI : cere_rotated_angle;
        cere_rotated_angle = cere_rotated_angle < -M_PI ? cere_rotated_angle + 2 * M_PI : cere_rotated_angle;
        tracker.angle = leaf_angle;
        tracker.pred_time -= (rclcpp::Time(data->header.stamp) - tracker.timestamp).seconds();
        tracker.timestamp = data->header.stamp;
        tracker.pred_angle = Integral(
            a_omega_phi_b[1],
            std::vector<double> { a_omega_phi_b[0],
                                  a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                  a_omega_phi_b[3] },
            (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
            tracker.pred_time
        );
        RCLCPP_DEBUG(node_->get_logger(), "rune_leaf change!");
    }
#endif
    CeresProcess(data, runes_msg, debug_msg);
    return true;
}

bool Tracker::CeresProcess(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    if (cere_param_list.size() < 100) {
        debug_msg.origin_big_rune_speed = leaf_angular_velocity = Revise(leaf_angle - leaf_angle_last, 0, 36_deg / 2) / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp)).seconds();
        DataProcess(data, debug_msg);
        runes_msg.can_shoot = false;
        return false;
    } else if (cere_param_list.size() == 100) {
        //队列数据已满
        cere_param_list.pop_front(); //队列头数据弹出
        //TODO:这里可能会有问题后续逻辑得仔细考虑一下
        debug_msg.origin_big_rune_speed = leaf_angular_velocity = Revise(leaf_angle - leaf_angle_last, 0, 36_deg / 2) / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp)).seconds();
        DataProcess(data, debug_msg);
        RCLCPP_DEBUG(node_->get_logger(), "finish_fitting flag %d", finish_fitting);
        //当现在的时间减去上一次拟合的时间大于预测的时间时，开始验证预测的准确性
        if ((rclcpp::Time(data->header.stamp) - rclcpp::Time(tracker.timestamp)).seconds() >= tracker.pred_time) {
            // 计算误差
            RCLCPP_DEBUG(node_->get_logger(), "calculate error");
            double delta_angle = 0;
            if (this->rotation_direction_ == RotationDirection::ANTICLOCKWISE && tracker.pred_angle > 0)
            {
                tracker.pred_angle *= -1;
            }
            //posteriori_angle 为预测的角度
            double posteriori_angle = (tracker.angle + tracker.pred_angle) > M_PI ? (tracker.angle + tracker.pred_angle) - 2 * M_PI : (tracker.angle + tracker.pred_angle);
            posteriori_angle = posteriori_angle < -M_PI ? posteriori_angle + 2 * M_PI : posteriori_angle;
            delta_angle = fabs(leaf_angle - posteriori_angle);
            delta_angle = delta_angle > M_PI ? fabs(2 * M_PI - delta_angle) : delta_angle;
            debug_msg.delta_angle = delta_angle;
            if (delta_angle < 0.10) {
                //误差小,则认为拟合良好
                RCLCPP_DEBUG(node_->get_logger(), "error small");
                pred_angle = Integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0],
                                          a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                          a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    delay + (node_->now() - data->header.stamp).seconds()
                );
                runes_msg.can_shoot = true; //可以发射
                count_cere = 0;             //将count_cere置为0，非连续5次拟合不良
                finish_fitting = true;
                tracker.Record(pred_angle, data->header.stamp, delay + (node_->now() - data->header.stamp).seconds(), leaf_angle);
            } else {
                RCLCPP_DEBUG(node_->get_logger(), "error big");
                //误差大,则认为拟合不良
                if (count_cere < 5) {
                    count_cere++;
                    pred_angle = Integral(
                        a_omega_phi_b[1],
                        std::vector<double> { a_omega_phi_b[0],
                                              a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                              a_omega_phi_b[3] },
                        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                        delay + (node_->now() - data->header.stamp).seconds()
                    );
                    tracker.Record(pred_angle, data->header.stamp, delay + (node_->now() - data->header.stamp).seconds(), leaf_angle);
                    return false;
                }
                finish_fitting = false; //连续五次误差超过0.1，则认为需要重新拟合
                count_cere = 0;
                Refitting();
                pred_angle = Integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1], a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    (delay + (node_->now() - data->header.stamp).seconds())
                );
                tracker.Record(pred_angle, data->header.stamp, delay + (node_->now() - rclcpp::Time(data->header.stamp)).seconds(), leaf_angle);
                runes_msg.can_shoot = false;
                return true;
            }
        } else {
            //还没有到达预测的时间
            pred_angle = Integral(
                a_omega_phi_b[1],
                std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1], a_omega_phi_b[3] },
                (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                (delay + (node_->now() - data->header.stamp).seconds())
            );
            return false;
        }
    }
    return true;
}

void Tracker::DataProcess(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    //如果这一帧和上一针时间差大于0.15s，则认为这一帧的数据不可用
    if ((rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp))
            .seconds()
        > 0.15) {
        count_cant_use = filter_astring_threshold; //因为卡尔曼滤波在数据突变后需要一定时间收敛，所以设置20个数据的收敛间隔
    }
    count_cant_use--;
#if NEW_METHOD
    MeasurementPackage package = MeasurementPackage(
        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
        MeasurementPackage::SensorType::LASER,
        Eigen::Vector2d { leaf_angular_velocity, 0 }
    );
    //直接输入角速度，然后进行滤波
    ukf_->ProcessMeasurement(package); //估计当前真实的状态
    double&& omega = 1.0 * abs(ukf_->x_(0));
    debug_msg.big_rune_speed = omega;
#else
    // 用二维坐标拟合
    auto&& theta = leaf_angle;
    MeasurementPackage package = MeasurementPackage(
        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
        MeasurementPackage::SensorType::LASER,
        Eigen::Vector2d { RUNE_ARMOR_TO_SYMBOL * cos(AngleRevise(theta, cere_rotated_angle)),
                          RUNE_ARMOR_TO_SYMBOL * sin(AngleRevise(theta, cere_rotated_angle)) }
    );
    debug_msg.small_rune_speed = AngleRevise(theta, cere_rotated_angle); //大符模式下用小符角速度查看符叶角度变化曲线
    //将传感器的坐标数据丢入UKF
    //ukf输入坐标，输出估计的状态向量x_为[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad

    ukf_->ProcessMeasurement(package);                              //估计当前真实的状态
    double&& omega = 1.0 * abs(ukf_->x_(2)) / RUNE_ARMOR_TO_SYMBOL; //从状态估计器中取出估计的omega
    debug_msg.big_rune_speed = omega;
#endif
    if (count_cant_use <= 0) {
        cere_param_list.push_back(CereParam {
            .omega = omega,
            .time = (rclcpp::Time(data->header.stamp) - t_zero).seconds() });
    }
}

void Tracker::Refitting() {
    RCLCPP_DEBUG(node_->get_logger(), "refitting");
    ceres::Problem problem;
    for (auto& i: cere_param_list) {
        //将数据添加入问题
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                new CURVE_FITTING_COST(
                    i.time,
                    i.omega
                )
            ),
            new ceres::CauchyLoss(0.4),
            a_omega_phi_b
        );
    }
    problem.SetParameterLowerBound(a_omega_phi_b, 0, 0.78); //设置参数上下限
    problem.SetParameterUpperBound(a_omega_phi_b, 0, 1.045);
    problem.SetParameterLowerBound(a_omega_phi_b, 1, 1.884);
    problem.SetParameterUpperBound(a_omega_phi_b, 1, 2); // 数据是官方的

    problem.SetParameterLowerBound(a_omega_phi_b, 2, -1 * M_PI);
    problem.SetParameterUpperBound(a_omega_phi_b, 2, 1 * M_PI);
    problem.SetParameterLowerBound(a_omega_phi_b, 3, 1.045);
    problem.SetParameterUpperBound(a_omega_phi_b, 3, 1.310);

    ceres::Solve(options, &problem, &summary); //开始拟合(解决问题)
}

bool Tracker::Judge(auto_aim_interfaces::msg::DebugRune& debug_msg) {
    static constexpr double delta = 1e-2;
    if (rotation_direction_ == RotationDirection::ANTICLOCKWISE ? leaf_angle_diff < delta
                                                                : leaf_angle_diff < -delta) {
        if (SetRotate(RotationDirection::ANTICLOCKWISE)) {
            debug_msg.rotation_direction = "Anticlockwise";
            return false;
        }
    } else if (rotation_direction_ == RotationDirection::CLOCKWISE ? leaf_angle_diff > -delta : leaf_angle_diff > delta) {
        if (SetRotate(RotationDirection::CLOCKWISE)) {
            debug_msg.rotation_direction = "Clockwise";
            return false;
        }
    } else {
        if (SetRotate(RotationDirection::STATIC)) {
            debug_msg.rotation_direction = "Static";
            return false;
        }
    }
    return false;
}

bool Tracker::Fitting(auto_aim_interfaces::msg::Target& runes_msg) {
    switch (motion_state_) {
        case MotionState::STATIC: {
            rotate_angle = 0;
        } break;
        case MotionState::SMALL: {
            switch (rotation_direction_) {
                case RotationDirection::CLOCKWISE: {
                    rotate_angle = speed.Mean() * delay;
                    runes_msg.can_shoot = true;
                } break;
                case RotationDirection::ANTICLOCKWISE: {
                    rotate_angle = -speed.Mean() * delay;
                    runes_msg.can_shoot = true;
                } break;
                default: {
                    runes_msg.can_shoot = false;
                    return false;
                }
            }
        } break;
        case MotionState::BIG: {
            switch (rotation_direction_) {
                case RotationDirection::CLOCKWISE: {
                    rotate_angle = pred_angle;
                } break;
                case RotationDirection::ANTICLOCKWISE: {
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

} // namespace rune
