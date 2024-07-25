#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace armor {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options):
    Node("armor_detector", options) {
    detector_ = CreateDetector();
    InitMarkers();

    // 申明参数
    show_image_ = declare_parameter("show_pic", false);
    debug_ = declare_parameter("debug", false);
    camera_coordinate_ = declare_parameter("camera_coordinate", "camera");

    // 监视 Debug 参数变化
    if (debug_) {
        CreateDebugPublishers();
    }
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
        "debug",
        [this](const rclcpp::Parameter& param) {
            debug_ = param.as_bool();
            debug_ ? CreateDebugPublishers() : DestroyDebugPublishers();
        }
    );

    armors_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

    ignore_classes_sub_ = create_subscription<auto_aim_interfaces::msg::IgnoreClasses>(
        "/detector/ignore_classes",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::IgnoreClasses::SharedPtr msg) {
            detector_->UpdateIgnoreClasses(msg->ignore_classes);
        }
    );

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_for_armor",
        rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&ArmorDetectorNode::ImageCallback, this, std::placeholders::_1)
    );

    last_publish_time_ = this->now();
}

void ArmorDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 颜色信息借用图像信息的 frame_id 传递，具体看相机节点
    // !比赛时一定要写死颜色，不要相信下位机的颜色数据！
    detector_->UpdateEnemyColor(msg->header.frame_id == "0" ? Color::RED : Color::BLUE);
    msg->header.frame_id = camera_coordinate_;
    auto&& raw_image = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
    std::vector<Armor> armors;

    if (debug_) {
        UpdateDetectorParameters();
        armors = detector_->DetectArmor(raw_image);

        result_image_ = raw_image.clone();
        detector_->DrawResult(result_image_);
        PublishDebugInfo(armors, msg->header);
    } else {
        armors = detector_->DetectArmor(raw_image);
    }

    PublishArmors(armors, msg->header);
}

std::unique_ptr<Detector> ArmorDetectorNode::CreateDetector() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int&& binary_threshold_red = declare_parameter("binary_threshold_red", 100, param_desc);
    int&& light_contour_threshold_red = declare_parameter("light_contour_threshold_red", 100, param_desc);
    int&& binary_threshold_blue = declare_parameter("binary_threshold_blue", 100, param_desc);
    int&& light_contour_threshold_blue = declare_parameter("light_contour_threshold_blue", 100, param_desc);
    DetectorParam detector_param {
        { binary_threshold_red,
          binary_threshold_blue },
        { light_contour_threshold_red,
          light_contour_threshold_blue }
    };

    auto&& confidence_threshold = declare_parameter("confidence_threshold", 0.7);
    auto&& camera_matrix = declare_parameter("camera_matrix", std::vector<double> {});
    auto&& distortion_coefficients = declare_parameter("distortion_coefficients", std::vector<double> {});
    auto&& pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    auto&& model_path = declare_parameter("model_path", "/model/mlp.onnx");
    auto&& label_path = declare_parameter("label_path", "/model/label.txt");

    return std::make_unique<Detector>(
        detector_param,
        pkg_path + model_path,
        pkg_path + label_path,
        confidence_threshold,
        camera_matrix,
        distortion_coefficients
    );
}

void ArmorDetectorNode::CreateDebugPublishers() {
    debug_lights_pub_ = create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
    debug_armors_pub_ = create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);
    armor_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
    if (show_image_) {
        binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
        number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }
}

void ArmorDetectorNode::DestroyDebugPublishers() {
    debug_lights_pub_.reset();
    debug_armors_pub_.reset();
    armor_marker_pub_.reset();
    if (show_image_) {
        binary_img_pub_.shutdown();
        number_img_pub_.shutdown();
        result_img_pub_.shutdown();
    }
}

void ArmorDetectorNode::UpdateDetectorParameters() {
    int&& binary_threshold_red = get_parameter("binary_threshold_red").as_int();
    int&& light_contour_threshold_red = get_parameter("light_contour_threshold_red").as_int();
    int&& binary_threshold_blue = get_parameter("binary_threshold_blue").as_int();
    int&& light_contour_threshold_blue = get_parameter("light_contour_threshold_blue").as_int();

    // clang-format off
    detector_->UpdateDetectorParam(
        DetectorParam {
            { binary_threshold_red, binary_threshold_blue},
            { light_contour_threshold_red, light_contour_threshold_blue }
        }
    );
    // clang-format on
}

void ArmorDetectorNode::PublishDebugInfo(const std::vector<Armor>& armors, const std_msgs::msg::Header& header) {
    if (show_image_) {
        binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", detector_->GetBinaryImage()).toImageMsg());
        number_img_pub_.publish(cv_bridge::CvImage(header, "mono8", detector_->GetAllNumbersImage()).toImageMsg());
        result_img_pub_.publish(cv_bridge::CvImage(header, "bgr8", result_image_).toImageMsg());
    }

    auto_aim_interfaces::msg::DebugArmors debug_armors_msg;
    auto_aim_interfaces::msg::DebugLights debug_lights_msg;
    visualization_msgs::msg::MarkerArray marker_array;
    auto&& debug_lights = detector_->GetDebugLights();
    auto&& debug_armors = detector_->GetDebugArmors();

    for (const auto& light: debug_lights) {
        auto_aim_interfaces::msg::DebugLight debug_light_msg;
        debug_light_msg.set__tilt_angle(light.tilt_angle);
        debug_light_msg.set__center_x(light.center.x);
        debug_light_msg.set__ratio(light.ratio);
        debug_light_msg.set__is_light(light.valid);
        debug_lights_msg.data.push_back(debug_light_msg);
    }

    for (const auto& armor: debug_armors) {
        auto_aim_interfaces::msg::DebugArmor debug_armor_msg;
        debug_armor_msg.set__center_x(armor.center.x);
        debug_armor_msg.set__type(ARMOR_TYPE_STR[static_cast<int>(armor.type)]);
        debug_armor_msg.set__light_center_distance(armor.light_center_distance);
        debug_armor_msg.set__classification_result(armor.classification_result);
        debug_armor_msg.set__light_height_ratio(armor.light_height_ratio);
        debug_armor_msg.set__light_angle_diff(armor.light_angle_diff);
        debug_armor_msg.set__angle(armor.angle);
        debug_armors_msg.data.push_back(debug_armor_msg);
    }

    armor_marker_.header = text_marker_.header = header;
    for (const auto& armor: armors) {
        // Fill the markers
        if (armor.type != ArmorType::INVALID) {
            auto&& armor_pose_msg = GetPoseMsg(armor.pose);
            armor_marker_.id++;
            armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
            armor_marker_.pose = armor_pose_msg;
            text_marker_.id++;
            text_marker_.pose.position = armor_pose_msg.position;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = armor.classification_result;
            marker_array.markers.emplace_back(armor_marker_);
            marker_array.markers.emplace_back(text_marker_);
        }
    }

    debug_lights_pub_->publish(debug_lights_msg);
    debug_armors_pub_->publish(debug_armors_msg);
    armor_marker_pub_->publish(marker_array);
}

void ArmorDetectorNode::PublishArmors(const std::vector<Armor>& armors, const std_msgs::msg::Header& header) {
    auto_aim_interfaces::msg::Armors armors_msg;
    armors_msg.header = header;
    for (const auto& armor: armors) {
        auto_aim_interfaces::msg::Armor armor_msg;
        armor_msg.set__number(armor.number);
        armor_msg.set__type(ARMOR_TYPE_STR[static_cast<int>(armor.type)]);
        armor_msg.set__distance_to_image_center(armor.distance_to_image_center);
        armor_msg.set__pose(GetPoseMsg(armor.pose));
        armors_msg.armors.push_back(armor_msg);
    }

    armors_pub_->publish(armors_msg);

    rclcpp::Time&& now = this->now();
    RCLCPP_DEBUG(this->get_logger(), "fps: %f", 1.0 / (now - last_publish_time_).seconds());
    last_publish_time_ = now;
}

geometry_msgs::msg::Pose ArmorDetectorNode::GetPoseMsg(const ArmorPose& pose) const {
    geometry_msgs::msg::Pose pose_msg;

    // 平移
    pose_msg.position.x = pose.position.at<double>(0);
    pose_msg.position.y = pose.position.at<double>(1);
    pose_msg.position.z = pose.position.at<double>(2);

    // 旋转向量 to 旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(pose.rotation_vector, rotation_matrix);
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
    // tf2 旋转矩阵 to tf2 四元数
    tf2::Quaternion tf2_q;
    tf2_rotation_matrix.getRotation(tf2_q);
    pose_msg.orientation = tf2::toMsg(tf2_q);

    return pose_msg;
}

void ArmorDetectorNode::InitMarkers() {
    armor_marker_.ns = "armors";
    armor_marker_.id = 0;
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
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(armor::ArmorDetectorNode)
