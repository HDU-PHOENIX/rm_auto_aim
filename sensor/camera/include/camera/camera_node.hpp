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
    /**
     * @brief 发布图像
     */
    void LoopForPublish();

    /**
     * @brief 通信节点模式切换的回调
     * 
     * @param request 
     * @param response 
     */
    void ServiceCB(
        communicate::srv::ModeSwitch::Request::ConstSharedPtr request,
        communicate::srv::ModeSwitch::Response::SharedPtr response
    );

    /**
     * @brief 获取图像保存到 frame，从相机或者视频流
     */
    void GetImg();

    /**
     * @brief 开启内录节点
     */
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
