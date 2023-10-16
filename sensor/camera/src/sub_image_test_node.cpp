#include "opencv2/opencv.hpp"

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ImageSubscriberTest: public rclcpp::Node {
public:
    explicit ImageSubscriberTest(const rclcpp::NodeOptions& options):
        Node("image_subscriber_test", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        RCLCPP_INFO(this->get_logger(), "ImageSubscriberTest constructor begin");
        // this->camera_subscriber = image_transport::CameraSubscriber(
        //     this,
        //     "/image_raw",
        //     std::bind(&ImageSubscriberTest::Callback, this, std::placeholders::_1, std::placeholders::_2),
        //     "raw"
        // );
        // this->img_sub_ = image_transport::create_subscription(
        //     this,
        //     "/image_raw",
        //     std::bind(&ImageSubscriberTest::ImageCallback1, this, std::placeholders::_1),
        //     "raw"
        // );
        // sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/image_raw",
        //     rclcpp::SensorDataQoS(),
        //     std::bind(&ImageSubscriberTest::ImageCallback, this, std::placeholders::_1)
        // );
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::SensorDataQoS(),
            [](sensor_msgs::msg::Image::UniquePtr msg) {
                // Create a cv::Mat from the image message (without copying).
                cv::Mat cv_mat(msg->width, msg->height, CV_8UC3, msg->data.data());
                // Annotate the image with the pid, pointer address, and the watermark text.
                std::cout << "ptr: " << msg.get() << std::endl;
                cv::imshow("image", cv_mat);
                cv::waitKeyEx(1);
            }
        );
    }

private:
    void ImageCallback(sensor_msgs::msg::Image::UniquePtr& msg) {
        auto now_time = rclcpp::Clock().now();
        cv::Mat cv_mat(msg->height, msg->width, CV_8UC3, msg->data.data());
        // cv::imshow("image", cv_bridge::toCvShare(msg, "bgr8")->image);
        RCLCPP_INFO(
            this->get_logger(),
            "image_transport Received image FPS: %lf \n msg: %d",
            1.0 / (now_time - last_time1).seconds(),
            msg.get()
        );
        last_time1 = rclcpp::Clock().now();
    }

    rclcpp::Time last_time1;
    rclcpp::Time last_time2;
    image_transport::Subscriber img_sub_;
    image_transport::CameraSubscriber camera_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ImageSubscriberTest)
