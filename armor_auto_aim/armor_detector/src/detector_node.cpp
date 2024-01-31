#include <auto_aim_interfaces/msg/detail/serial_info__struct.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace armor {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options):
    Node("armor_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

    // 初始化 Detector
    detector_ = InitDetector();

    // 创建装甲板消息发布器
    armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
        "/detector/armors",
        rclcpp::SensorDataQoS()
    );

    this->InitMarkers();

    // Debug 信息发布者
    debug_ = this->declare_parameter("debug", false);
    send_default_armor_ = this->declare_parameter("send_default_armor", false);
    if (debug_) {
        CreateDebugPublishers();
    }

    // 监视 Debug 参数变化
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
        debug_ = p.as_bool();
        debug_ ? CreateDebugPublishers() : DestroyDebugPublishers();
    });

    pnp_iterative = this->declare_parameter("pnp_iterative", true);
    // 创建相机信息订阅者
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        pnp_solver_ = std::make_unique<PnPSolver>(pnp_iterative, camera_info->k, camera_info->d);
        cam_info_sub_.reset();
    });

    // 创建图像订阅者
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_for_armor", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::ImageCallback, this, std::placeholders::_1));

    no_armor_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialInfo>("/shooter_info", rclcpp::SensorDataQoS());
}

void ArmorDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    auto armors = DetectArmors(img_msg);

    // PnP 求解
    if (pnp_solver_ != nullptr) {
        armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
        armors_msg_.armors.clear();
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;

        // 创建装甲板消息
        auto_aim_interfaces::msg::Armor armor_msg;
        // 遍历装甲板进行 PnP 求解
        for (const auto& armor: armors) {
            // 旋转、平移向量
            cv::Mat rvec, tvec;
            bool success = pnp_solver_->SolvePnP(armor, rvec, tvec);
            if (success) {
                // 写入装甲板基础消息
                armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
                armor_msg.number = armor.number;

                // 写入 PnP 求解结果
                armor_msg.pose.position.x = tvec.at<double>(0);
                armor_msg.pose.position.y = tvec.at<double>(1);
                armor_msg.pose.position.z = tvec.at<double>(2);
                // 旋转向量 to 旋转矩阵
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                // tf2 旋转矩阵
                tf2::Matrix3x3 tf2_rotation_matrix(
                    rotation_matrix.at<double>(0, 0),
                    rotation_matrix.at<double>(0, 1),
                    rotation_matrix.at<double>(0, 2),
                    rotation_matrix.at<double>(1, 0),
                    rotation_matrix.at<double>(1, 1),
                    rotation_matrix.at<double>(1, 2),
                    rotation_matrix.at<double>(2, 0),
                    rotation_matrix.at<double>(2, 1),
                    rotation_matrix.at<double>(2, 2)
                );
                // 旋转矩阵 to 四元数
                tf2::Quaternion tf2_q;
                tf2_rotation_matrix.getRotation(tf2_q);
                armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                // 计算装甲板中心到图像中心的距离
                armor_msg.distance_to_image_center = pnp_solver_->CalculateDistanceToCenter(armor.center);

                // Fill the markers
                armor_marker_.id++;
                armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
                armor_marker_.pose = armor_msg.pose;
                text_marker_.id++;
                text_marker_.pose.position = armor_msg.pose.position;
                text_marker_.pose.position.y -= 0.1;
                text_marker_.text = armor.classfication_result;
                armors_msg_.armors.emplace_back(armor_msg);
                marker_array_.markers.emplace_back(armor_marker_);
                marker_array_.markers.emplace_back(text_marker_);
            } else {
                RCLCPP_WARN(this->get_logger(), "PnP failed!");
            }
        }

        if (armors_msg_.armors.size() > 0) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Find " << armors_msg_.armors.size() << " armors!");
            armors_pub_->publish(armors_msg_);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No armor found!");
            auto_aim_interfaces::msg::SerialInfo no_armor_msg;
            no_armor_msg.start.set__data('s');
            no_armor_msg.end.set__data('e');
            no_armor_msg.euler = { 0, 0, 0 };
            no_armor_pub_->publish(no_armor_msg);
        }

        // Publishing marker
        PublishMarkers();
    } else {
        RCLCPP_ERROR(this->get_logger(), "PnP solver not initialized!");
    }
}

std::unique_ptr<Detector> ArmorDetectorNode::InitDetector() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = declare_parameter("binary_thres", 160, param_desc);

    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int gray_thres = declare_parameter("gray_thres", 160, param_desc);

    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int color_thres = declare_parameter("color_thres", 160, param_desc);

    Detector::LightParams l_params;
    param_desc.integer_range.clear();
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.1;
    param_desc.floating_point_range[0].from_value = 0;
    param_desc.floating_point_range[0].to_value = 1;
    l_params.min_ratio = declare_parameter("light.min_ratio", 0.1, param_desc);
    l_params.max_ratio = declare_parameter("light.max_ratio", 0.4, param_desc);
    param_desc.floating_point_range.clear();
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 90;
    l_params.max_angle = declare_parameter("light.max_angle", 40.0, param_desc);

    Detector::ArmorParams a_params;
    param_desc.integer_range.clear();
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.1;
    param_desc.floating_point_range[0].from_value = 0;
    param_desc.floating_point_range[0].to_value = 1;
    a_params.min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7);
    a_params.min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8);
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.1;
    param_desc.floating_point_range[0].from_value = 0;
    param_desc.floating_point_range[0].to_value = 10;
    a_params.max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2, param_desc);
    a_params.min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2, param_desc);
    a_params.max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5, param_desc);
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.5;
    param_desc.floating_point_range[0].from_value = 0;
    param_desc.floating_point_range[0].to_value = 90;
    a_params.max_angle = declare_parameter("armor.max_angle", 35.0);

    param_desc.description = "0-RED, 1-BLUE";
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    int enemy_color = declare_parameter("enemy_color", RED, param_desc);

    param_desc.description = "0-普通二值化，1-通道相减二值化";
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    int detect_mode = declare_parameter("detect_mode", 0, param_desc);

    // 初始化 Detector
    std::unique_ptr<Detector> detector;
    if (detect_mode == 0) {
        detector = std::make_unique<Detector>(binary_thres, enemy_color, l_params, a_params, detect_mode);
    } else if (detect_mode == 1) {
        detector = std::make_unique<Detector>(gray_thres, color_thres, enemy_color, l_params, a_params, detect_mode);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid detect_mode!");
    }

    // 初始化分类器
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    auto model_path = pkg_path + "/model/mlp.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    double threshold = this->declare_parameter("classifier_threshold", 0.7);
    std::vector<std::string> ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string> { "negative" });
    detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
}

std::vector<Armor> ArmorDetectorNode::DetectArmors(const sensor_msgs::msg::Image::SharedPtr& img_msg) {
    // 通过 img_msg 指针构造图像
    auto&& img = cv::Mat(img_msg->height, img_msg->width, CV_8UC3, img_msg->data.data());

    // 实时更新识别器参数
    if (this->debug_) {
        detector_->binary_thres = get_parameter("binary_thres").as_int();
        detector_->gray_thres = get_parameter("gray_thres").as_int();
        detector_->color_thres = get_parameter("color_thres").as_int();
        detector_->enemy_color = get_parameter("enemy_color").as_int();
        detector_->detect_mode = get_parameter("detect_mode").as_int();
        detector_->light_params = {
            get_parameter("light.min_ratio").as_double(),
            get_parameter("light.max_angle").as_double(),
            get_parameter("light.max_angle").as_double()
        };
        detector_->armor_params = {
            get_parameter("armor.min_light_ratio").as_double(),
            get_parameter("armor.min_small_center_distance").as_double(),
            get_parameter("armor.max_small_center_distance").as_double(),
            get_parameter("armor.min_large_center_distance").as_double(),
            get_parameter("armor.max_large_center_distance").as_double(),
            get_parameter("armor.max_angle").as_double()
        };
        detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();
    }

    // 检测装甲板
    auto armors = detector_->Detect(img);

    // 计算检测时间
    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

    // 发布 debug 信息
    if (debug_) {
        PublishDebugInfo(img, img_msg, armors, latency);
    }

    return armors;
}

void ArmorDetectorNode::CreateDebugPublishers() {
    lights_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
    armors_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

    gray_mask_pub_ = image_transport::create_publisher(this, "/detector/gray_mask");
    color_mask_pub_ = image_transport::create_publisher(this, "/detector/color_mask");
    binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
    number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
    result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::DestroyDebugPublishers() {
    // 重置
    lights_data_pub_.reset();
    armors_data_pub_.reset();

    // 关闭
    gray_mask_pub_.shutdown();
    color_mask_pub_.shutdown();
    binary_img_pub_.shutdown();
    number_img_pub_.shutdown();
    result_img_pub_.shutdown();
}

void ArmorDetectorNode::PublishDebugInfo(cv::Mat& img, const sensor_msgs::msg::Image::SharedPtr& img_msg, const std::vector<Armor>& armors, const double& latency) {
    if (detector_->detect_mode == 1) {
        gray_mask_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->gray_mask).toImageMsg());
        color_mask_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->color_mask).toImageMsg());
    }
    binary_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // 根据 x 坐标对灯条和装甲板排序
    std::sort(detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(), [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
    std::sort(detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(), [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

    // 发布灯条和装甲板 debug 信息
    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
        // 获取所有的数字图像
        auto all_num_img = detector_->GetAllNumbersImage();
        // 发布数字图像
        number_img_pub_.publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    // 在图像上画出结果，显示数字和置信度
    detector_->DrawResults(img);
    // 画出相机中心
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    // 发布结果图像
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "bgr8", img).toImageMsg());
}

void ArmorDetectorNode::InitMarkers() {
    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
}

void ArmorDetectorNode::PublishMarkers() {
    // using Marker = visualization_msgs::msg::Marker;
    // armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorDetectorNode)
