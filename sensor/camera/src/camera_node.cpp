#include "camera/camera_node.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>

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
    image_publisher_for_armor_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_armor",
        rclcpp::SensorDataQoS()
    );
    image_publisher_for_rune_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_rune",
        rclcpp::SensorDataQoS()
    );

    // 创建订阅者
    serial_info_subscriber_ = this->create_subscription<auto_aim_interfaces::msg::SerialInfo>(
        "/serial_info",
        rclcpp::SensorDataQoS(),
        std::bind(&CameraNode::SerialInfoCallback, this, std::placeholders::_1)
    );

    video_writer_ = std::make_shared<cv::VideoWriter>();
    if (inner_shot_flag) {
        video_writer_->open("./Camera/inner_shot.mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(1280, 1024));
    }
}

CameraNode::~CameraNode() {
    video_writer_->release();
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void CameraNode::SerialInfoCallback(const auto_aim_interfaces::msg::SerialInfo::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "isopened %d", video_writer_->isOpened());
    sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
    image_msg->header.stamp = this->now();
    if (videoflag) {
        capture >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "video end");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
        } else {
            image_msg->header.frame_id = "camera";
            image_msg->height = frame.rows;
            image_msg->width = frame.cols;
            image_msg->encoding = "bgr8";
            image_msg->is_bigendian = 0u;
            image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            image_msg->data.assign(frame.datastart, frame.dataend);

            // 根据 msg->mode.data 的值，选择发布到哪个话题
            if (msg->mode.data == 'a') {
                // RCLCPP_INFO(this->get_logger(), "publish image for armor");
                image_publisher_for_armor_->publish(std::move(image_msg));
            } else if (msg->mode.data == 'r') {
                // RCLCPP_INFO(this->get_logger(), "publish image for rune");
                //0 为不可激活，1 为小符，2 为大符 将图片的 frame_id 临时设置为大小符模式，接收端要再改回来
                if (msg->rune_flag.data == 0) {
                    image_msg->header.frame_id = "0";
                } else if (msg->rune_flag.data == 1) {
                    image_msg->header.frame_id = "1";
                } else if (msg->rune_flag.data == 2) {
                    image_msg->header.frame_id = "2";
                }
                image_publisher_for_rune_->publish(std::move(image_msg));
            } else {
                RCLCPP_ERROR(this->get_logger(), "mode should be a or r but got %c", msg->mode.data);
            }
        }
    } else {
        // 从 MindVision 摄像头获取图像
        if (!this->GetFrame(frame_)) {
            RCLCPP_ERROR(this->get_logger(), "get image failed");
        }
        if (inner_shot_flag) {
            video_writer_->write(*frame_);
        }

        // RCLCPP_ERROR(this->get_logger(), "get image success");
        image_msg->header.frame_id = "camera";
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);

        // 根据 msg->mode.data 的值，选择发布到哪个话题
        if (msg->mode.data == 'a') {
            // RCLCPP_INFO(this->get_logger(), "publish image for armor");
            image_publisher_for_armor_->publish(std::move(image_msg));
        } else if (msg->mode.data == 'r') {
            // RCLCPP_INFO(this->get_logger(), "publish image for rune");
            //0 为不可激活，1 为小符，2 为大符 将图片的 frame_id 临时设置为大小符模式，接收端要再改回来
            if (msg->rune_flag.data == 0) {
                image_msg->header.frame_id = "0";
            } else if (msg->rune_flag.data == 1) {
                image_msg->header.frame_id = "1";
            } else if (msg->rune_flag.data == 2) {
                image_msg->header.frame_id = "2";
            }
            image_publisher_for_rune_->publish(std::move(image_msg));
        } else {
            RCLCPP_ERROR(this->get_logger(), "mode should be a or r but got %c", msg->mode.data);
        }
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
