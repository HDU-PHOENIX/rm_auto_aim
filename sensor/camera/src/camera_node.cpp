#include "camera/camera_node.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <unistd.h>

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    //是否使用视频流标志位
    videoflag = this->declare_parameter("videoflag", false);
    video_path = this->declare_parameter("video_path", "/home/robot/1.avi"); //默认路径
    inner_shot_flag = this->declare_parameter("inner_shot_flag", false);

    if (this->videoflag) {
        capture.open(video_path);
    }
    // 创建发布者
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_pub",
        rclcpp::SensorDataQoS()
    );

    video_writer_ = std::make_shared<cv::VideoWriter>();
    if (inner_shot_flag) {
        video_writer_->open("./Camera/inner_shot.mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(1280, 1024));
    }
    frame_ = std::make_shared<cv::Mat>();
    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

CameraNode::~CameraNode() {
    video_writer_->release();
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
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
}

void CameraNode::LoopForPublish() {
    // RCLCPP_INFO(this->get_logger(), "isopened %d", video_writer_->isOpened());
    while (rclcpp::ok()) {
        usleep(100);
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        image_msg->header.stamp = this->now();
        this->GetImg();
        image_msg->header.frame_id = "camera";
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);
        if (inner_shot_flag) {
            video_writer_->write(*frame_);
        }
        image_publisher_->publish(std::move(image_msg));
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
