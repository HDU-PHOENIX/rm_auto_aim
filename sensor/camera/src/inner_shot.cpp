#include "camera/inner_shot.hpp"

namespace sensor {
InnerShotNode::InnerShotNode():
    Node("innershot_node") {
    RCLCPP_INFO(this->get_logger(), "innershot_node start");
    video_writer_ = std::make_shared<cv::VideoWriter>();
    img_inner_shot_sub_ = this->create_subscription<auto_aim_interfaces::msg::Image>("/image_pub", rclcpp::SensorDataQoS(), std::bind(&InnerShotNode::InnerShotCallback, this, std::placeholders::_1));
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&t);
    std::string name;
    name = std::to_string(now_tm->tm_year + 1900) + "-" + std::to_string(now_tm->tm_mon + 1) + "-" + std::to_string(now_tm->tm_mday) + "-" + std::to_string(now_tm->tm_hour) + "-" + std::to_string(now_tm->tm_min) + "-" + std::to_string(now_tm->tm_sec);
    video_writer_->open("./Camera/" + name + ".mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 60, cv::Size(1280, 720));
}

void InnerShotNode::InnerShotCallback(const auto_aim_interfaces::msg::Image::SharedPtr img_msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg->raw_image, sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat& img = cv_ptr->image;
    video_writer_->write(img);
}
InnerShotNode::~InnerShotNode() {
    video_writer_->release();
}
} // namespace sensor