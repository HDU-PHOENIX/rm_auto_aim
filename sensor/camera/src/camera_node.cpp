#include "camera/camera_node.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    canceled_(false) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");
    // 创建发布者
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS());

    // 创建线程，绑定 LoopForPublish
    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

CameraNode::~CameraNode() {
    // 等待线程结束
    canceled_.store(true);
    if (thread_for_publish_.joinable()) {
        thread_for_publish_.join();
    }

    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void CameraNode::LoopForPublish() {
    while (rclcpp::ok() && !canceled_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // auto start = this->now();
        // std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
        // cv::Mat frame;
        // 从 MindVision 摄像头获取图像
        if (!this->GetFrame(frame_)) {
            RCLCPP_WARN(this->get_logger(), "get image failed");
        }
        // cv::imshow("test", (*frame_));
        // cv::waitKey(1);
        // auto end = this->now();
        // std::cout << "Get image" << (end - start).seconds() * 1000 << "ms" << std::endl;
        // std::cout << "毫秒:"
        //           << std::chrono::duration_cast<std::chrono::milliseconds>(
        //                  std::chrono::system_clock::now() - start
        //              )
        //                  .count()
        //           << std::endl;

        // 创建 Image 消息的 UniquePtr msg
        // 向 msg 中填充图像数据
        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->height = frame_->rows;
        msg->width = frame_->cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = 0u;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        msg->data.assign(frame_->datastart, frame_->dataend);

        // 发布消息
        this->publisher_->publish(std::move(msg));
        // auto end2 = this->now();
        // std::cout << "except get image" << (end2 - end).seconds() * 1000 << "ms" << std::endl;
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
