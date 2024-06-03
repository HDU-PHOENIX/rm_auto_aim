#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "camera/mindvision.hpp"
#include "inner_shot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <communicate/srv/mode_switch.hpp>
#include <opencv2/videoio.hpp>
namespace sensor {

class CameraNode: public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);

private:
    void LoopForPublish(); //发布图像

    void ServiceCB(const std::shared_ptr<communicate::srv::ModeSwitch::Request> request, std::shared_ptr<communicate::srv::ModeSwitch::Response> response);

    void GetImg(); //获取图像

    void InnerShot();

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    rclcpp::Service<communicate::srv::ModeSwitch>::SharedPtr mode_switch_server_;
    std::string enemy_color_or_rune_flag;
    bool mode_; //决定图片往哪个topic发布 符或者装甲板 mode: rune true auto_aim false
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_for_rune_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_for_armor_;
    int rune_use_exposure_;

    // 是否外部输入视频流标志位
    bool videoflag;
    std::string video_path;
    std::shared_ptr<MindVision> mindvision_;
    cv::VideoCapture capture;
    std::thread thread_for_publish_;    //获取图像的线程
    std::thread thread_for_inner_shot_; //获取图像的线程
    bool inner_shot_flag;

    int failed_count;
};

} // namespace sensor

#endif // CAMERA_NODE_HPP
