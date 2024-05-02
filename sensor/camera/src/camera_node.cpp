#include "camera/camera_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    MindVision(ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config") {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    //是否使用视频流标志位
    videoflag = this->declare_parameter("videoflag", false);
    video_path = this->declare_parameter("video_path", "/home/robot/1.avi"); //默认路径
    rune_use_exposure_ = this->declare_parameter("rune_exposure", 4000);

    // euler_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //     "/communicate/gyro/left",
    //     rclcpp::SensorDataQoS().keep_last(2),
    //     std::bind(&CameraNode::EulerCallback, this, std::placeholders::_1)
    // );

    if (this->videoflag) {
        capture.open(video_path);
    }
    // 创建发布者
    image_publisher_ = this->create_publisher<auto_aim_interfaces::msg::Image>(
        "/camera/armor_image",
        rclcpp::SensorDataQoS().keep_last(2)
    );

    img_pub_for_rune_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_rune",
        rclcpp::SensorDataQoS().keep_last(2)
    );

    img_pub_for_armor_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_armor",
        rclcpp::SensorDataQoS().keep_last(2)
    );
    frame_ = std::make_shared<cv::Mat>();
    mode_ = false;
    mode_switch_server_ = this->create_service<communicate::srv::ModeSwitch>("/communicate/autoaim", std::bind(&CameraNode::ServiceCB, this, std::placeholders::_1, std::placeholders::_2));
    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

void CameraNode::ServiceCB(const std::shared_ptr<communicate::srv::ModeSwitch::Request> request, std::shared_ptr<communicate::srv::ModeSwitch::Response> response) {
    //模式 0：自瞄 1：符
    this->mode_ = request->mode == 0 ? false : true;
    if (mode_) {
        this->SetExposureTime(ament_index_cpp::get_package_share_directory("auto_aim") + "/config/rune_mindvision.config");
        this->SetExposureTime(rune_use_exposure_); //符曝光
    } else {
        //装甲板曝光
        this->SetExposureTime(ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config");
    }
    this->enemy_color_or_rune_flag = request->mode == 0 ? std::to_string(request->enemy_color) : std::to_string(request->rune_state);
    response->success = true;
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

// void CameraNode::EulerCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
//     auto_aim_interfaces::msg::Image::UniquePtr image_msg(new auto_aim_interfaces::msg::Image());
//     image_msg->header.stamp = msg->header.stamp;
//     this->GetImg();
//     image_msg->header.frame_id = "camera";
//     image_msg->color = msg->header.frame_id;
//     image_msg->raw_image.height = frame_->rows;
//     image_msg->raw_image.width = frame_->cols;
//     image_msg->raw_image.encoding = "bgr8";
//     image_msg->raw_image.is_bigendian = 0u;
//     image_msg->raw_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
//     image_msg->raw_image.data.assign(frame_->datastart, frame_->dataend);

//     image_msg->yaw_and_pitch = {
//         static_cast<float>(msg->position[0]),
//         static_cast<float>(msg->position[1])
//     };

//     image_publisher_->publish(std::move(image_msg));
// }

void CameraNode::GetImg() {
    if (videoflag) {
        capture >> *frame_;
        if ((*frame_).empty()) {
            RCLCPP_INFO(this->get_logger(), "video end");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            capture >> *frame_;
        }
    } else {
        if (!this->GetFrame(frame_)) {
            RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
        }
    }
}

void CameraNode::LoopForPublish() {
    while (rclcpp::ok()) {
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        this->GetImg();
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = this->enemy_color_or_rune_flag;
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);
        mode_ ? img_pub_for_rune_->publish(std::move(image_msg)) : img_pub_for_armor_->publish(std::move(image_msg));
        // RCLCPP_INFO(this->get_logger(), "publish image");
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
