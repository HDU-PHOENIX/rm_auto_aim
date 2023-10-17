#include "camera/subscriber_test_node.hpp"
#include <opencv2/highgui.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

SubscriberTestNode::SubscriberTestNode():
    Node("image_subscriber_test", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    RCLCPP_INFO(this->get_logger(), "ImageSubscriberTest constructor begin");
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::UniquePtr msg) {
            auto receive_begin_time = rclcpp::Clock().now();
            cv::Mat cv_mat(msg->height, msg->width, CV_8UC3, msg->data.data());
            // cv::imshow("image", cv_mat);
            // cv::waitKey(1);
            auto receive_end_time = rclcpp::Clock().now();
            RCLCPP_INFO(
                this->get_logger(),
                "Receive FPS: %f, FPS between two receive %f, address %p",
                1.0 / (receive_end_time - receive_begin_time).seconds(),
                1.0 / (receive_end_time - last_time).seconds(),
                static_cast<void*>(msg.get())
            );
            last_time = rclcpp::Clock().now();
        }
    );
}
