#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "auto_aim_interfaces/msg/image.hpp"
#include "camera/inner_shot.hpp"
#include "camera/mindvision.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace sensor {

class CameraNode: public rclcpp::Node, MindVision {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;

private:
    // void LoopForPublish(); //发布图像

    void InnerShot(); //内部录制视频

    void GetImg(); //获取图像

    void EulerCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr euler_sub_;

    // 原始图像发布者
    rclcpp::Publisher<auto_aim_interfaces::msg::Image>::SharedPtr image_publisher_;

    rclcpp::Publisher<auto_aim_interfaces::msg::Image>::SharedPtr img_inner_shot_;

    //是否外部输入视频流标志位
    bool videoflag;
    bool inner_shot_flag;
    std::string video_path;
    cv::VideoCapture capture;
    std::thread thread_for_publish_;    //获取图像的线程
    std::thread thread_for_inner_shot_; //获取图像的线程
};

} // namespace sensor

#endif // CAMERA_NODE_HPP
