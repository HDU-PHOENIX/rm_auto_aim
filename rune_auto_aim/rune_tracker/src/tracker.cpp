#include "rune_tracker/tracker.hpp"
#include <rclcpp/logger.hpp>
#define NEW_METHOD true
namespace rune {
Tracker::Tracker(double&& std_a_, double&& std_yawdd_, int& filter_astring_threshold_) {
    ukf_ = new UKF_PLUS(false, true, false, std_a_, std_yawdd_);
    this->filter_astring_threshold = filter_astring_threshold_;
    InitCeres(); //��ʼ��ceres
}

void Tracker::Predict(const auto_aim_interfaces::msg::Rune::SharedPtr& data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    auto&& theory_delay = data->pose_c.position.z / data->speed;
    runes_msg.delay = delay = theory_delay + data->chasedelay;
    runes_msg.header = data->header; //ʱ�����ֵ
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
    } //����λ���������ݣ��ж��Ǿ�ֹ����С�����Ǵ��

    cv::Point2f tmp_dir(data->leaf_dir.x, data->leaf_dir.y); //���ĸ������ĵ�R��
    leaf_angle = Angle(std::move(tmp_dir));                  //���ػ����ƵĽǶ�
    CalSmallRune(data, debug_msg);                           //����С�����ٶ�
    Judge(debug_msg);                                        //�ж�˳ʱ�뻹����ʱ��
    FittingBig(data, runes_msg, debug_msg);                  //��ϴ��
    Fitting(runes_msg);                                      //����Ԥ��Ƕ�
    data_last = data;                                        //��¼��һ֡������
    leaf_angle_last = leaf_angle;                            //��¼��һ֡�ĽǶ�
    cv::Point2d symbol(data->symbol.x, data->symbol.y);      //R��
    rotate_armors.clear();
    for (int i = 0; i < 4; i++) {
        rotate_armors.emplace_back(data->rune_points[i].x, data->rune_points[i].y);
    }
    for (auto&& vertex: rotate_armors) {
        //���ؼ�����Բ����תrotate_angle �õ�Ԥ���
        vertex = Rotate(vertex, symbol, rotate_angle);
    }
}

void Tracker::InitCeres() {
    finish_fitting = false;
    tracker.pred_time = 0;
    tracker.pred_angle = 0;
    tracker.angle = 0;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //ѡ����С���˵����ģʽ
    options.minimizer_progress_to_stdout = false;              //ѡ�񲻴�ӡ�����Ϣ
    options.num_threads = 4;                                   //ʹ��4���߳̽������
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

    if (angles.Any()) {
        leaf_angle_diff = Revise(leaf_angle - leaf_angle_last, -36_deg, 36_deg); //������Χ
        auto&& leaf_angle_diff_abs = std::abs(leaf_angle_diff);
        angles.PushForcibly(angles[-1] + leaf_angle_diff_abs);
        //���������ٶ�
        speed.Push(
            leaf_angle_diff_abs
            / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp))
                  .seconds()
        );
        speeds.PushForcibly(speed.Value());
    } else {
        angles.PushForcibly(leaf_angle);
    }
    debug_msg.small_rune_speed = speed.Mean();
}

bool Tracker::FittingBig(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    if (motion_state_ != MotionState::BIG) {
        return false;
    }

    if (cere_param_list.empty()) {              //���ݶ���Ϊ��ʱ����ʼ��
        tracker.timestamp = data->header.stamp; //��¼ʱ���
        t_zero = data->header.stamp;            //ʱ�����
        cere_rotated_angle = leaf_angle;        //��¼��һ֡��Ҷ�ĽǶ�
        tracker.pred_time = 0;
        tracker.angle = cere_rotated_angle;
    }

    if ((rclcpp::Time(data->header.stamp) - t_zero).seconds() > 30) {
        //�������ݹ���
        Reset();
        return false;
    }
    if (abs(leaf_angle - leaf_angle_last) > 0.4) {
        //��һ֡����һ֡�ĽǶȲ�ֵ����0.4�����ж�Ϊ�ɼ���ķ�Ҷ��ת��
        cere_rotated_angle = leaf_angle - leaf_angle_last + cere_rotated_angle; // �任��Ҷ��ʼ�Ƕ�
        //�Ƕȱ任����Ҫ�����Ƕ�
        cere_rotated_angle = cere_rotated_angle > M_PI ? cere_rotated_angle - 2 * M_PI : cere_rotated_angle;
        cere_rotated_angle = cere_rotated_angle < -M_PI ? cere_rotated_angle + 2 * M_PI : cere_rotated_angle;
        // RCLCPP_INFO(this->get_logger(), "rune_leaf change!");
    }
    CeresProcess(data, runes_msg, debug_msg);
    return true;
}

bool Tracker::CeresProcess(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::Target& runes_msg, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    if (cere_param_list.size() < 100) {
        leaf_angular_velocity = fabs(leaf_angle_diff) / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp)).seconds();
        DataProcess(data, debug_msg);
        runes_msg.can_shoot = false;
        return false;
    } else if (cere_param_list.size() == 100) {
        //������������
        cere_param_list.pop_front(); //����ͷ���ݵ���
        //TODO:������ܻ�����������߼�����ϸ����һ��
        leaf_angular_velocity = fabs(leaf_angle_diff) / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp)).seconds();
        DataProcess(data, debug_msg);
        if (finish_fitting) {
            RCLCPP_INFO(rclcpp::get_logger("tracker"), "predict correct");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("tracker"), "predict inaccuracy,restart fittting");
        }
        //�����ڵ�ʱ���ȥ��һ����ϵ�ʱ�����Ԥ���ʱ��ʱ����ʼ��֤Ԥ���׼ȷ��
        if ((rclcpp::Time(data->header.stamp) - rclcpp::Time(tracker.timestamp)).seconds() >= tracker.pred_time) {
            // �������
            double delta_angle = 0;
            if (this->rotation_direction_ == RotationDirection::ANTICLOCKWISE && tracker.pred_angle > 0)
            {
                tracker.pred_angle *= -1;
            }
            delta_angle = fabs(leaf_angle - (tracker.angle + tracker.pred_angle));
            delta_angle = delta_angle > M_PI ? fabs(2 * M_PI - delta_angle) : delta_angle;
            debug_msg.delta_angle = delta_angle;
            if (delta_angle < 0.2) {
                //���С,����Ϊ�������
                pred_angle = Integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0],
                                          a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                          a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    delay + (rclcpp::Clock().now() - data->header.stamp).seconds()
                );
                runes_msg.can_shoot = true; //���Է���
                count_cere = 0;             //��count_cere��Ϊ0��������5����ϲ���
                finish_fitting = true;
                tracker.Record(pred_angle, data->header.stamp, delay + (rclcpp::Clock().now() - data->header.stamp).seconds(), leaf_angle);
            } else {
                //����,����Ϊ��ϲ���
                if (count_cere < 5) {
                    count_cere++;
                    pred_angle = Integral(
                        a_omega_phi_b[1],
                        std::vector<double> { a_omega_phi_b[0],
                                              a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                              a_omega_phi_b[3] },
                        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                        delay + (rclcpp::Clock().now() - data->header.stamp).seconds()
                    );
                    tracker.Record(pred_angle, data->header.stamp, delay + (rclcpp::Clock().now() - data->header.stamp).seconds(), leaf_angle);
                    return false;
                }
                finish_fitting = false; //�����������0.1������Ϊ��Ҫ�������
                count_cere = 0;
                Refitting(data);
                runes_msg.can_shoot = true;
                return true;
            }
        } else {
            //��û�е���Ԥ���ʱ��
            pred_angle = Integral(
                a_omega_phi_b[1],
                std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1], a_omega_phi_b[3] },
                (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                (delay + (rclcpp::Clock().now() - data->header.stamp).seconds())
            );
            runes_msg.can_shoot = false;
            return false;
        }
    }
    return true;
}

void Tracker::DataProcess(auto_aim_interfaces::msg::Rune::SharedPtr data, auto_aim_interfaces::msg::DebugRune& debug_msg) {
    if (leaf_angular_velocity < 4) {
        //�����һ֡����һ��ʱ������0.15s������Ϊ��һ֡�����ݲ�����
        if ((rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp))
                .seconds()
            > 0.15) {
            count_cant_use = filter_astring_threshold; //��Ϊ�������˲�������ͻ�����Ҫһ��ʱ����������������20�����ݵ��������
        }
        count_cant_use--;
#if NEW_METHOD
        MeasurementPackage package = MeasurementPackage(
            (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
            MeasurementPackage::SensorType::LASER,
            Eigen::Vector2d { leaf_angular_velocity, 0 }
        );
        //ֱ��������ٶȣ�Ȼ������˲�
        ukf_->ProcessMeasurement(package); //���Ƶ�ǰ��ʵ��״̬
        double&& omega = 1.0 * abs(ukf_->x_(0));
        debug_msg.big_rune_speed = omega;
#else
        // �ö�ά�������
        auto&& theta = leaf_angle;
        MeasurementPackage package = MeasurementPackage(
            (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
            MeasurementPackage::SensorType::LASER,
            Eigen::Vector2d { RUNE_ARMOR_TO_SYMBOL * cos(theta - cere_rotated_angle),
                              RUNE_ARMOR_TO_SYMBOL * sin(theta - cere_rotated_angle) }
        );
        //�����������������ݶ���UKF
        //ukf�������꣬������Ƶ�״̬����x_Ϊ[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad

        ukf_->ProcessMeasurement(package);                              //���Ƶ�ǰ��ʵ��״̬
        double&& omega = 1.0 * abs(ukf_->x_(2)) / RUNE_ARMOR_TO_SYMBOL; //��״̬��������ȡ�����Ƶ�omega
        debug_msg.big_rune_speed = omega;
#endif
        if (count_cant_use <= 0) {
            cere_param_list.push_back(CereParam {
                .omega = omega,
                .time = (rclcpp::Time(data->header.stamp) - t_zero).seconds() });
        }
    }
}

void Tracker::Refitting(auto_aim_interfaces::msg::Rune::SharedPtr data) {
    ceres::Problem problem;
    for (auto& i: cere_param_list) {
        //���������������
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                new CURVE_FITTING_COST(
                    i.time,
                    i.omega
                )
            ),
            new ceres::CauchyLoss(0.1),
            a_omega_phi_b
        );
    }
    problem.SetParameterLowerBound(a_omega_phi_b, 0, 0.78); //���ò���������
    problem.SetParameterUpperBound(a_omega_phi_b, 0, 1.045);
    problem.SetParameterLowerBound(a_omega_phi_b, 1, 1.884);
    problem.SetParameterUpperBound(a_omega_phi_b, 1, 2); // �����ǹٷ���

    problem.SetParameterLowerBound(a_omega_phi_b, 2, -1 * M_PI);
    problem.SetParameterUpperBound(a_omega_phi_b, 2, 1 * M_PI);
    problem.SetParameterLowerBound(a_omega_phi_b, 3, 1.045);
    problem.SetParameterUpperBound(a_omega_phi_b, 3, 1.310);

    ceres::Solve(options, &problem, &summary); //��ʼ���(�������)
    pred_angle = Integral(
        a_omega_phi_b[1],
        std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1], a_omega_phi_b[3] },
        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
        (delay + (rclcpp::Clock().now() - data->header.stamp).seconds())
    );
    tracker.Record(pred_angle, data->header.stamp, delay + (rclcpp::Clock().now() - rclcpp::Time(data->header.stamp)).seconds(), leaf_angle);
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

double
Tracker::Integral(double w, std::vector<double> params, double t_s, double pred_time) {
    double a = params[0];
    double phi = params[1];
    double t_e = t_s + pred_time;
    double theta1 = -a / w * cos(w * t_s + phi) + (params[2]) * t_s;
    double theta2 = -a / w * cos(w * t_e + phi) + (params[2]) * t_e;
    return theta2 - theta1;
} //���� ����Ԥ���¸�ʱ�̵�λ��

} // namespace rune
