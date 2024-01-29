#include "rune_tracker/tracker_node.hpp"
#define NEW_METHOD    true
#define PNP_ITERATION true
namespace rune {
// RuneTrackerNode类的构造函数
RuneTrackerNode::RuneTrackerNode(const rclcpp::NodeOptions& option):
    Node("rune_tracker", option) {
    // 打印信息，表示节点已启动
    RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //选择最小二乘的拟合模式
    finish_fitting = false;
    tracker.pred_time = 0;
    tracker.pred_angle = 0;
    tracker.angle = 0;
    options.minimizer_progress_to_stdout = false; //选择不打印拟合信息
    options.num_threads = 4;                      //使用4个线程进行拟合
    a_omega_phi_b[0] = RUNE_ROTATE_A_MIN;
    a_omega_phi_b[1] = RUNE_ROTATE_O_MIN;
    a_omega_phi_b[2] = 0;
    a_omega_phi_b[3] = 0;
    count_cere = 0;
    bullet_speed = this->declare_parameter("bullet_speed", 25.0);
    filter_astring_threshold = this->declare_parameter("filter_astring_threshold", 0);

    this->declare_parameter("chasedelay", 0.0); //设置chasedelay的默认值0.0

    this->declare_parameter("phase_offset", 0.0); //设置phaseoffset的默认值0.0

    CreateDebugPublisher(); //创建Debug发布器
    //ukf滤波器初始化 1.5  1.2
    tracker_ = std::make_unique<Tracker>(1.5, 1.2); //tracker中的ukf滤波器初始化

    // 创建相机信息订阅者
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        // RCLCPP_INFO(this->get_logger(), "camera_info received");
        cam_info_sub_.reset();
    });

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
    target_frame_ = this->declare_parameter("target_frame", "shooter");
    tf2_filter_ = std::make_shared<tf2_filter>(
        runes_sub_,                         // message_filters subscriber
        *tf2_buffer_,                       // tf2 buffer
        target_frame_,                      // frame this filter should attempt to transform to
        100,                                // size of the tf2 cache
        this->get_node_logging_interface(), // node logging interface
        this->get_node_clock_interface(),   // node clock interface
        std::chrono::duration<int>(1)       // timeout
    );
    // 注册回调函数
    tf2_filter_->registerCallback(&RuneTrackerNode::RunesCallback, this);

    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::RuneTarget>(
        "/RuneTracker2Shooter",
        rclcpp::SensorDataQoS()
    );
    InitRecord(); //打开txt文件 用于记录卡尔曼滤波曲线和原始曲线

    // 可视化标记发布器
    // 参见 http://wiki.ros.org/rviz/DisplayTypes/Marker
    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03; //TODO:x,y,z值还需修改
    armor_marker_.scale.y = 0.23;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rune_tracker/marker", 10);
}

void RuneTrackerNode::InitRecord() {
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
    origin_omega_time.open("./record/origin_omegatime.txt");
    if (!origin_omega_time.is_open()) {
        while (true) {
            std::cout << "cannot open the origin_omegatime.txt" << std::endl;
        }
    }
    origin_omega_file.open("./record/origin_omega.txt");
    if (!origin_omega_file.is_open()) {
        while (true) {
            std::cout << "cannot open the origin_omega.txt" << std::endl;
        }
    }
}

bool RuneTrackerNode::Judge() {
    constexpr double delta = 1e-2;
    if (rotation_direction == RotationDirection::Anticlockwise ? leaf_angle_diff < delta
                                                               : leaf_angle_diff < -delta) {
        if (SetRotate(RotationDirection::Anticlockwise)) {
            return false;
        }
    } else if (rotation_direction == RotationDirection::Clockwise ? leaf_angle_diff > -delta : leaf_angle_diff > delta) {
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
    RCLCPP_INFO(this->get_logger(), "fitting big");
    if (cere_param_list.empty()) {              //数据队列为空时，初始化
        tracker.timestamp = data->header.stamp; //记录时间戳
        t_zero = data->header.stamp;            //时间起点
        cere_rotated_angle = leaf_angle;        //记录第一帧符叶的角度
        tracker.pred_time = 0;
        tracker.angle = cere_rotated_angle;
    }

    if ((rclcpp::Time(data->header.stamp) - t_zero).seconds() > 30) {
        //符的数据过期
        Reset();
        return false;
    }
    if (abs(leaf_angle - leaf_angle_last) > 0.4) {
        //上一帧与这一帧的角度差值超过0.4，则判断为可激活的符叶已转换
        cere_rotated_angle = leaf_angle - leaf_angle_last + cere_rotated_angle; // 变换符叶初始角度
        //角度变换后，需要矫正角度
        cere_rotated_angle = cere_rotated_angle > M_PI ? cere_rotated_angle - 2 * M_PI : cere_rotated_angle;
        cere_rotated_angle = cere_rotated_angle < -M_PI ? cere_rotated_angle + 2 * M_PI : cere_rotated_angle;
        RCLCPP_INFO(this->get_logger(), "rune_leaf change!");
    }
    CeresProcess();
    return true;
}

void RuneTrackerNode::DataProcess() {
    if (leaf_angular_velocity < 4) {
        origin_omega_file << leaf_angular_velocity << std::endl;
        origin_omega_time << (rclcpp::Time(data->header.stamp) - t_zero).seconds() << std::endl;
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
        tracker_->ukf->ProcessMeasurement(package); //估计当前真实的状态
        double&& omega = 1.0 * abs(tracker_->ukf->x_(0));
#else
        // 用二维坐标拟合
        auto&& theta = leaf_angle;
        MeasurementPackage package = MeasurementPackage(
            (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
            MeasurementPackage::SensorType::LASER,
            Eigen::Vector2d { RUNE_ARMOR_TO_SYMBOL * cos(theta - cere_rotated_angle),
                              RUNE_ARMOR_TO_SYMBOL * sin(theta - cere_rotated_angle) }
        );
        //将传感器的坐标数据丢入UKF
        //ukf输入坐标，输出估计的状态向量x_为[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad

        tracker_->ukf->ProcessMeasurement(package);                              //估计当前真实的状态
        double&& omega = 1.0 * abs(tracker_->ukf->x_(2)) / RUNE_ARMOR_TO_SYMBOL; //从状态估计器中取出估计的omega
#endif
        if (count_cant_use <= 0) {
            cere_param_list.push_back(CereParam {
                .omega = omega,
                .time = (rclcpp::Time(data->header.stamp) - t_zero).seconds() });

            omega_file << omega << std::endl;
            omega_time << (rclcpp::Time(data->header.stamp) - t_zero).seconds() << std::endl;
        }
    }
}

bool RuneTrackerNode::CeresProcess() {
    if (cere_param_list.size() < 100) {
        //数据队列设为100个，数据队列未满
        leaf_angular_velocity = fabs(leaf_angle_diff) / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp)).seconds();
        DataProcess();
        runes_msg_.can_shoot = false;
        return false;
    } else if (cere_param_list.size() == 100) {
        //队列数据已满
        cere_param_list.pop_front(); //队列头数据弹出
        //TODO:这里可能会有问题后续逻辑得仔细考虑一下
        leaf_angular_velocity = fabs(leaf_angle_diff) / (rclcpp::Time(data->header.stamp) - rclcpp::Time(data_last->header.stamp)).seconds();
        DataProcess();

        if (finish_fitting) {
            RCLCPP_INFO(this->get_logger(), "predict correct");
        } else {
            RCLCPP_INFO(this->get_logger(), "predict inaccuracy,restart fittting");
        }
        //当现在的时间减去上一次拟合的时间大于预测的时间时，开始验证预测的准确性
        if ((rclcpp::Time(data->header.stamp) - rclcpp::Time(tracker.timestamp)).seconds()
            >= tracker.pred_time) {
            // 计算误差
            double delta_angle = 0;
            if (this->rotation_direction == RotationDirection::Anticlockwise && tracker.pred_angle > 0) {
                tracker.pred_angle *= -1;
            }
            delta_angle = fabs(leaf_angle - (tracker.angle + tracker.pred_angle));
            delta_angle = delta_angle > M_PI ? fabs(2 * M_PI - delta_angle) : delta_angle;
            debug_msg_.delta_angle = delta_angle;
            if (delta_angle < 0.2) {
                //误差小,则认为拟合良好
                pred_angle = Integral(
                    a_omega_phi_b[1],
                    std::vector<double> { a_omega_phi_b[0],
                                          a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                          a_omega_phi_b[3] },
                    (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                    delay + (this->now() - data->header.stamp).seconds()
                );
                runes_msg_.can_shoot = true; //可以发射
                count_cere = 0;              //将count_cere置为0，非连续5次拟合不良
                finish_fitting = true;
                tracker.pred_angle = pred_angle;
                tracker.timestamp = data->header.stamp;
                tracker.pred_time = delay + (this->now() - data->header.stamp).seconds();
                tracker.angle = leaf_angle; //记录当前角度 用于后续的拟合检测
            } else {
                //误差大,则认为拟合不良
                if (count_cere < 5) {
                    count_cere++;
                    pred_angle = Integral(
                        a_omega_phi_b[1],
                        std::vector<double> { a_omega_phi_b[0],
                                              a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1],
                                              a_omega_phi_b[3] },
                        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                        delay + (this->now() - data->header.stamp).seconds()
                    );
                    tracker.pred_angle = pred_angle;
                    tracker.timestamp = data->header.stamp;
                    tracker.pred_time = delay + (this->now() - data->header.stamp).seconds();
                    tracker.angle = leaf_angle; //记录当前角度 用于后续的拟合检测
                    return false;
                }
                finish_fitting = false; //连续五次误差超过0.1，则认为需要重新拟合
                count_cere = 0;
                Refitting();
                return true;
            }
        } else {
            //还没有到达预测的时间
            pred_angle = Integral(
                a_omega_phi_b[1],
                std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1], a_omega_phi_b[3] },
                (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
                (delay + (this->now() - data->header.stamp).seconds())
            );
            runes_msg_.can_shoot = false;
            return false;
        }
    }
    return true;
}

void RuneTrackerNode::Refitting() {
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
            new ceres::CauchyLoss(0.1),
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
    pred_angle = Integral(
        a_omega_phi_b[1],
        std::vector<double> { a_omega_phi_b[0], a_omega_phi_b[2] + phase_offset * a_omega_phi_b[1], a_omega_phi_b[3] },
        (rclcpp::Time(data->header.stamp) - t_zero).seconds(),
        (delay + (this->now() - data->header.stamp).seconds())
    );
    tracker.pred_time = delay + (this->now() - rclcpp::Time(data->header.stamp)).seconds();
    tracker.pred_angle = pred_angle;
    tracker.angle = leaf_angle;
    tracker.timestamp = data->header.stamp;
    runes_msg_.can_shoot = true;
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
RuneTrackerNode::Integral(double w, std::vector<double> params, double t_s, double pred_time) {
    double a = params[0];
    double phi = params[1];
    double t_e = t_s + pred_time;
    double theta1 = -a / w * cos(w * t_s + phi) + (params[2]) * t_s;
    double theta2 = -a / w * cos(w * t_e + phi) + (params[2]) * t_e;
    return theta2 - theta1;
} //积分 用于预测下个时刻的位置

// runeCallback函数实现 接收rune_detector发布的rune消息
void RuneTrackerNode::RunesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr) {
    data = rune_ptr;
    auto&& theory_delay = data->pose_c.position.z / bullet_speed;
    chasedelay = this->get_parameter("chasedelay").as_double();
    delay = theory_delay + chasedelay;
    runes_msg_.speed = bullet_speed;
    runes_msg_.delay = delay;
    runes_msg_.header = data->header; //时间戳赋值
    phase_offset = this->get_parameter("phase_offset").as_double();
    // if (data->motion == 0) {
    //     SetState(MotionState::Static);
    // } else if (data->motion == 1) {
    //     SetState(MotionState::Small);
    // } else if (data->motion == 2) {
    //     SetState(MotionState::Big);
    // } //从下位机来的数据，判断是静止还是小符还是大符

    SetState(MotionState::Small);

    cv::Point2f tmp_dir(data->leaf_dir.x, data->leaf_dir.y); //符四个点中心到R标

    this->leaf_dir = std::move(tmp_dir); //现在这一帧符叶向量

    leaf_angle = Angle(leaf_dir);                       //返回弧度制的角度
    CalSmallRune();                                     //计算小符角速度
    Judge();                                            //判断顺时针还是逆时针
    FittingBig();                                       //拟合大符
    Fitting();                                          //计算预测角度
    data_last = data;                                   //记录上一帧的数据
    leaf_angle_last = leaf_angle;                       //记录上一帧的角度
    std::vector<cv::Point2d> rotate_armors;             //旋转后的装甲板坐标
    cv::Point2d symbol(data->symbol.x, data->symbol.y); //R标
    for (int i = 0; i < 4; i++) {
        rotate_armors.emplace_back(data->rune_points[i].x, data->rune_points[i].y);
    }
    for (auto&& vertex: rotate_armors) {
        //将关键点以圆心旋转rotate_angle 得到预测点
        vertex = Rotate(vertex, symbol, rotate_angle);
    }
    cv::Mat rvec, tvec; //tvec为旋转后的相机坐标系下的坐标
    if (pnp_solver_->SolvePnP(rotate_armors, rvec, tvec, PNP_ITERATION)) {
    } else {
        RCLCPP_INFO(this->get_logger(), "rune_tracker solve pnp failed");
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header = data->header;
    ps.pose.position.x = runes_msg_.pc.position.x = tvec.at<double>(0);
    ps.pose.position.y = runes_msg_.pc.position.y = tvec.at<double>(1);
    ps.pose.position.z = runes_msg_.pc.position.z = tvec.at<double>(2);
    try {
        // 将装甲板位置从 相机坐标系 转换到 shooter（目标坐标系）
        // 此后装甲板信息中的 armor.pose 为装甲板在 shooter 系中的位置
        ps.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException& ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming  %s", ex.what());
        return;
    }
    runes_msg_.pw.position.x = ps.pose.position.x;
    runes_msg_.pw.position.y = ps.pose.position.y;
    runes_msg_.pw.position.z = ps.pose.position.z;
    target_pub_->publish(runes_msg_);
    PublishMarkers(runes_msg_);
    PublishDebugInfo();
}

void RuneTrackerNode::CalSmallRune() {
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
    debug_msg_.small_rune_speed = speed.Mean();
}

void RuneTrackerNode::PublishMarkers(const auto_aim_interfaces::msg::RuneTarget& target_msg) {
    armor_marker_.header = target_msg.header;
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.id = 0;
    armor_marker_.pose.position.x = target_msg.pc.position.x;
    armor_marker_.pose.position.y = target_msg.pc.position.y;
    armor_marker_.pose.position.z = target_msg.pc.position.z; //可视化相机坐标系下的预测的装甲板位置
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.emplace_back(armor_marker_);
    armor_marker_.header.frame_id = target_frame_;
    armor_marker_.id = 1;
    armor_marker_.pose.position.x = target_msg.pw.position.x;
    armor_marker_.pose.position.y = target_msg.pw.position.y;
    armor_marker_.pose.position.z = target_msg.pw.position.z; //可视化odom坐标系下的装甲板位置
    marker_array.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array);
}

void RuneTrackerNode::CreateDebugPublisher() {
    debug_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugRune>(
        "/rune_tracker/debug",
        rclcpp::SensorDataQoS()
    );
}
void RuneTrackerNode::PublishDebugInfo() {
    debug_msg_.delay = delay;
    debug_msg_.rotate_angle = rotate_angle;
    debug_msg_.a_omega_phi_b[0] = a_omega_phi_b[0];
    debug_msg_.a_omega_phi_b[1] = a_omega_phi_b[1];
    debug_msg_.a_omega_phi_b[2] = a_omega_phi_b[2];
    debug_msg_.a_omega_phi_b[3] = a_omega_phi_b[3];
    debug_pub_->publish(debug_msg_);
}
} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// 用class_loader注册组件。
// 这充当一种入口点，允许在将其库加载到运行中的进程时发现组件。
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneTrackerNode)
