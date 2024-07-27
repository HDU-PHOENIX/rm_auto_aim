#include "camera/camera_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    frame_(std::make_shared<cv::Mat>()),
    mode_(false),
    failed_count(0) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    //是否使用视频流标志位
    videoflag = this->declare_parameter("videoflag", false);
    video_path = this->declare_parameter("video_path", "/home/robot/1.avi"); //默认路径
    rune_use_exposure_ = this->declare_parameter("rune_exposure", 4000);
    inner_shot_flag = this->declare_parameter("inner_shot_flag", false);

    RCLCPP_INFO(this->get_logger(), "inner_shot flag %d", inner_shot_flag);

    mindvision_ = std::make_shared<MindVision>(
        ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config",
        this->declare_parameter("sn", "").c_str()
    );

    if (videoflag) {
        capture.open(video_path);
        if (!capture.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "video open failed");
            exit(-1);
        }
        RCLCPP_INFO(this->get_logger(), "use video");
    } else if (!mindvision_->GetCameraStatus()) {
        RCLCPP_ERROR(this->get_logger(), "mindvision failed");
        exit(-1);
    }

    if (inner_shot_flag) {
        thread_for_inner_shot_ = std::thread(std::bind(&CameraNode::InnerShot, this));
    }

    img_pub_for_rune_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_rune",
        rclcpp::SensorDataQoS().keep_last(2)
    );
    img_pub_for_armor_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_armor",
        rclcpp::SensorDataQoS().keep_last(2)
    );
    mode_switch_server_ = this->create_service<communicate::srv::ModeSwitch>(
        "/communicate/autoaim",
        std::bind(&CameraNode::ServiceCB, this, std::placeholders::_1, std::placeholders::_2)
    );

    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

void CameraNode::InnerShot() {
    auto inner_shot = std::make_shared<InnerShotNode>();
    RCLCPP_INFO(this->get_logger(), "inner_shot start !.............. ");
    rclcpp::spin(inner_shot);
}

void CameraNode::ServiceCB(
    communicate::srv::ModeSwitch::Request::ConstSharedPtr request,
    communicate::srv::ModeSwitch::Response::SharedPtr response
) {
    //模式 0：自瞄 1：符
    this->mode_ = request->mode == 0 ? false : true;
    if (mode_) {
        // 符曝光
        mindvision_->SetExposureTime(
            ament_index_cpp::get_package_share_directory(
                "auto_aim"
            )
            + "/config/rune_mindvision.config"
        );
        mindvision_->SetExposureTime(rune_use_exposure_);
    } else {
        //装甲板曝光
        mindvision_->SetExposureTime(
            ament_index_cpp::get_package_share_directory(
                "auto_aim"
            )
            + "/config/mindvision.config"
        );
    }

    this->enemy_color_or_rune_flag =
        request->mode == 0 ? std::to_string(request->enemy_color)
                           : std::to_string(request->rune_state);
    response->success = true;
}

void CameraNode::GetImg() {
    if (videoflag) {
        capture >> *frame_;

        // 循环播放
        if ((*frame_).empty()) {
            RCLCPP_INFO(this->get_logger(), "video end");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            capture >> *frame_;
        }
    } else {
        if (!mindvision_->GetFrame(frame_)) {
            failed_count++;
            RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
        } else {
            failed_count = 0;
        }
    }

    if (failed_count > 10) {
        exit(-1);
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
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
