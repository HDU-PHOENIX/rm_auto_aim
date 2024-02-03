#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "auto_aim_interfaces/msg/serial_info.hpp"
#include "camera/mindvision.hpp"
#include <memory>
#include <opencv2/videoio.hpp>
#include <string>

namespace sensor {

class CameraNode: public rclcpp::Node, MindVision {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;

private:
    void LoopForPublish(); //发布图像

    void GetImg(); //获取图像

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;
    std::shared_ptr<cv::VideoWriter> video_writer_;

    // debug 时间数据
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    rclcpp::Time last_publish_time;

    // 原始图像发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    //是否外部输入视频流标志位
    bool videoflag;
    bool inner_shot_flag;
    std::string video_path;
    cv::VideoCapture capture;
    std::thread thread_for_publish_; //获取图像的线程
};

} // namespace sensor

#endif // CAMERA_NODE_HPP
