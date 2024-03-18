#include "camera/camera_node.hpp"
#include "camera/inner_shot.hpp"
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
    inner_shot_flag = this->declare_parameter("inner_shot_flag", false);

    euler_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/communicate/gyro/left",
        rclcpp::SensorDataQoS(),
        std::bind(&CameraNode::EulerCallback, this, std::placeholders::_1)
    );

    if (this->videoflag) {
        capture.open(video_path);
    }
    // 创建发布者
    image_publisher_ = this->create_publisher<auto_aim_interfaces::msg::Image>(
        "/image_pub",
        2
    );

    if (inner_shot_flag) {
        thread_for_inner_shot_ = std::thread(std::bind(&CameraNode::InnerShot, this));
    }
    frame_ = std::make_shared<cv::Mat>();
    // thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void CameraNode::InnerShot() {
    auto inner_shot = std::make_shared<InnerShotNode>();
    rclcpp::spin(inner_shot);
}

void CameraNode::EulerCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    auto_aim_interfaces::msg::Image::UniquePtr image_msg(new auto_aim_interfaces::msg::Image());
    image_msg->header.stamp = msg->header.stamp;
    this->GetImg();
    image_msg->header.frame_id = "camera";
    image_msg->raw_image.height = frame_->rows;
    image_msg->raw_image.width = frame_->cols;
    image_msg->raw_image.encoding = "bgr8";
    image_msg->raw_image.is_bigendian = 0u;
    image_msg->raw_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
    image_msg->raw_image.data.assign(frame_->datastart, frame_->dataend);

    image_msg->yaw_and_pitch = {
        static_cast<float>(msg->position[0]),
        static_cast<float>(msg->position[1])
    };

    image_publisher_->publish(std::move(image_msg));
}

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
            exit(-1);
        }
    }
    // cv::imshow("raw", *frame_);
    // cv::waitKey(1);
}

// void CameraNode::LoopForPublish() {
//     while (rclcpp::ok()) {
//         sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
//         image_msg->header.stamp = this->now();
//         this->GetImg();
//         image_msg->header.frame_id = "camera";
//         image_msg->height = frame_->rows;
//         image_msg->width = frame_->cols;
//         image_msg->encoding = "bgr8";
//         image_msg->is_bigendian = 0u;
//         image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
//         image_msg->data.assign(frame_->datastart, frame_->dataend);

//         image_publisher_->publish(std::move(image_msg));
//         // RCLCPP_INFO(this->get_logger(), "publish image");
//     }
// }

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
