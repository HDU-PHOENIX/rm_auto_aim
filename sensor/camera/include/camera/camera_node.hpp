#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "camera/mindvision.hpp"
#include <opencv2/videoio.hpp>
#include <std_msgs/msg/detail/int32_multi_array__struct.hpp>
#include <string>

namespace sensor {

class CameraNode: public rclcpp::Node, MindVision {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;
    void SerialInfoCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

private:
    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    // debug 时间数据
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    rclcpp::Time last_publish_time;

    // 原始图像发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_for_armor_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_for_rune_;
    // 串口数据接收者
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr serial_info_subscriber_;

    //是否外部输入视频流标志位
    bool videoflag;
    std::string video_path;
    cv::VideoCapture capture;
    cv::Mat frame;
};

} // namespace sensor

#endif // CAMERA_NODE_HPP
