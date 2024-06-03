#include "camera/inner_shot.hpp"

namespace sensor {
InnerShotNode::InnerShotNode():
    Node("innershot_node") {
    RCLCPP_INFO(this->get_logger(), "innershot_node start");
    video_writer_ = std::make_shared<cv::VideoWriter>();
    img_inner_shot_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_for_armor", rclcpp::SensorDataQoS().keep_last(2), std::bind(&InnerShotNode::InnerShotCallback, this, std::placeholders::_1));
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&t);
    std::string name;
    name = std::to_string(now_tm->tm_year + 1900) + "-" + std::to_string(now_tm->tm_mon + 1) + "-" + std::to_string(now_tm->tm_mday) + "-" + std::to_string(now_tm->tm_hour) + "-" + std::to_string(now_tm->tm_min) + "-" + std::to_string(now_tm->tm_sec);
    video_writer_->open("./Camera/" + name + ".mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 100, cv::Size(1280, 720));

    count_ = 0;
    video_count = 0;
}

void InnerShotNode::InnerShotCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    count_++;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat& img = cv_ptr->image;
    video_writer_->write(img);

    if (count_ > 1500) {
        video_writer_->release();
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&t);
        std::string name;
        name = std::to_string(now_tm->tm_year + 1900) + "-" + std::to_string(now_tm->tm_mon + 1) + "-" + std::to_string(now_tm->tm_mday) + "-" + std::to_string(now_tm->tm_hour) + "-" + std::to_string(now_tm->tm_min) + "-" + std::to_string(now_tm->tm_sec);
        video_writer_->open("./Camera/" + name + std::to_string(video_count) + ".mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 100, cv::Size(1280, 720));
        video_count++;
        count_ = 0;
    }
}
InnerShotNode::~InnerShotNode() {
    RCLCPP_INFO(this->get_logger(), "Innershot destruction");
    video_writer_->release();
}
} // namespace sensor