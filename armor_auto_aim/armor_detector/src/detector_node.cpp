#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace armor {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options):
    Node("armor_detector", options) {
    lost_count_ = 0;
    detector_ = CreateDetector();
    InitMarkers();

    // 监视 Debug 参数变化
    debug_ = declare_parameter("debug", false);
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
    no_armor_pub_ = create_publisher<communicate::msg::SerialInfo>("/shoot_info/left", 10);

    ignore_classes_sub_ = create_subscription<auto_aim_interfaces::msg::IgnoreClasses>(
        "/detector/ignore_classes",
        rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::IgnoreClasses::SharedPtr msg) {
            detector_->UpdateIgnoreClasses(msg->ignore_classes);
        }
    );

    last_publish_time_ = this->now();

    mode_info_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/communicate/autoaim",
        100,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            if (msg->data[1] == 0) {
                detector_->UpdateEnemyColor(msg->data[0] == 0 ? Color::RED : Color::BLUE);
                image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "/image_pub",
                    2,
                    std::bind(&ArmorDetectorNode::ImageCallback, this, std::placeholders::_1)
                );
            } else {
                image_sub_.reset();
            }
        }
    );
}

void ArmorDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
    auto&& binary_threshold = declare_parameter("binary_threshold", 100, param_desc);
    auto&& light_contour_threshold = declare_parameter("light_contour_threshold", 100, param_desc);

    // enemy_color 0-红色 1-蓝色，实际上是无效值
    // 没有收到通信节点的 communicate/autoaim 消息时，并不会进行识别
    // 收到消息后才启动图片订阅者，并更新 enemy_color
    auto&& enemy_color = declare_parameter("enemy_color", 0); // 0-红色 1-蓝色
    auto&& confidence_threshold = declare_parameter("confidence_threshold", 0.7);
    auto&& camera_matrix = declare_parameter("camera_matrix", std::vector<double> {});
    auto&& distortion_coefficients = declare_parameter("distortion_coefficients", std::vector<double> {});
    auto&& pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    auto&& model_path = declare_parameter("model_path", "/model/mlp.onnx");
    auto&& label_path = declare_parameter("label_path", "/model/label.txt");
    auto&& ignore_classes = declare_parameter("ignore_classes", std::vector<std::string> { "negative" });

    return std::make_unique<Detector>(
        binary_threshold,
        light_contour_threshold,
        enemy_color == 0 ? Color::RED : Color::BLUE,
        pkg_path + model_path,
        pkg_path + label_path,
        confidence_threshold,
        camera_matrix,
        distortion_coefficients,
        ignore_classes
    );
}

void ArmorDetectorNode::CreateDebugPublishers() {
    debug_lights_pub_ = create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
    debug_armors_pub_ = create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);
    armor_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
    binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
    number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
    result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::DestroyDebugPublishers() {
    debug_lights_pub_.reset();
    debug_armors_pub_.reset();
    armor_marker_pub_.reset();
    binary_img_pub_.shutdown();
    number_img_pub_.shutdown();
    result_img_pub_.shutdown();
}

void ArmorDetectorNode::UpdateDetectorParameters() {}

void ArmorDetectorNode::PublishDebugInfo(const std::vector<Armor>& armors, const std_msgs::msg::Header& header) {
    binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", detector_->GetBinaryImage()).toImageMsg());
    number_img_pub_.publish(cv_bridge::CvImage(header, "mono8", detector_->GetAllNumbersImage()).toImageMsg());
    result_img_pub_.publish(cv_bridge::CvImage(header, "bgr8", result_image_).toImageMsg());

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
        debug_armor_msg.set__light_height_ratio(armor.light_height_ratio);
        debug_armor_msg.set__light_angle_diff(armor.light_angle_diff);
        debug_armor_msg.set__angle(armor.angle);
        debug_armors_msg.data.push_back(debug_armor_msg);
    }

    armor_marker_.header = text_marker_.header = header;
    for (const auto& armor: armors) {
        // Fill the markers
        if (armor.type != ArmorType::INVALID) {
            // armor_marker_.id++;
            armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
            armor_marker_.pose.position.x = armor.position.x;
            armor_marker_.pose.position.y = armor.position.y;
            armor_marker_.pose.position.z = armor.position.z;
            text_marker_.id++;
            text_marker_.pose.position = armor_marker_.pose.position;
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
    if (!armors.empty()) {
        auto_aim_interfaces::msg::Armors armors_msg;
        armors_msg.header = header;
        for (const auto& armor: armors) {
            auto_aim_interfaces::msg::Armor armor_msg;
            armor_msg.set__number(armor.number);
            armor_msg.set__type(ARMOR_TYPE_STR[static_cast<int>(armor.type)]);
            armor_msg.set__distance_to_image_center(armor.distance_to_image_center);
            armor_msg.pose.position.set__x(armor.position.x);
            armor_msg.pose.position.set__y(armor.position.y);
            armor_msg.pose.position.set__z(armor.position.z);
            armors_msg.armors.push_back(armor_msg);
        }

        rclcpp::Time&& now = this->now();
        RCLCPP_DEBUG(this->get_logger(), "fps: %f", 1.0 / (now - last_publish_time_).seconds());
        last_publish_time_ = now;
        armors_pub_->publish(armors_msg);
        lost_count_ = 0;
    } else if (++lost_count_ > 5) {
        communicate::msg::SerialInfo no_armor_msg;
        no_armor_msg.start.set__data('s');
        no_armor_msg.end.set__data('e');
        no_armor_msg.is_find.set__data('0');
        no_armor_msg.can_shoot.set__data('0');
        no_armor_msg.euler = { 0, 0, 0 };
        no_armor_pub_->publish(no_armor_msg);

        RCLCPP_DEBUG(this->get_logger(), "No armor detected");
    }
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
