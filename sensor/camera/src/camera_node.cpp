#include "camera/mindvision.hpp"
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>

#include "camera/common.hpp"

namespace sensor {

class CameraNode: public rclcpp::Node, sensor::MindVision {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options):
        Node("camera_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        sensor_msgs::msg::Image tt;
        RCLCPP_INFO(this->get_logger(), "camera_node start");
        // camera_publisher_ = image_transport::create_camera_publisher(this, "/image_raw");
        publisher_ = image_transport::create_publisher(this, "/image_raw");
        while (this->GetFrame(cv_image)) {
            auto time_begin_while = rclcpp::Clock().now();

            RCLCPP_INFO(this->get_logger(), "1");
            // msg->header.frame_id = "camera_frame";
            RCLCPP_INFO(this->get_logger(), "11");

            tt.height = cv_image.rows;
            tt.width = cv_image.cols;
            tt.encoding = "bgr8";
            tt.is_bigendian = false;
            RCLCPP_INFO(this->get_logger(), "2");
            tt.step = static_cast<sensor_msgs::msg::Image::_step_type>(cv_image.step);
            RCLCPP_INFO(this->get_logger(), "3");
            tt.data.assign(cv_image.datastart, cv_image.dataend);
            RCLCPP_INFO(this->get_logger(), "4");
            // this->image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
            // this->image_msg->header.stamp = rclcpp::Clock().now();
            auto&& msg = std::make_unique<sensor_msgs::msg::Image>(tt);
            auto time_begin_publish = rclcpp::Clock().now();
            publisher_.publish(std::move(msg));
            RCLCPP_INFO(this->get_logger(), "5");
            // camera_publisher_.publish(*this->image_msg, this->camera_info_msg);
            auto time_end_publish = rclcpp::Clock().now();
            std::cout << "ptr: " << msg.get() << std::endl;
            RCLCPP_INFO(
                this->get_logger(),
                "publish fps: %f \n while fps %f",
                1.0 / (time_end_publish - time_begin_publish).seconds(),
                1.0 / (time_end_publish - time_begin_while).seconds()
            );
            RCLCPP_INFO(
                this->get_logger(),
                "publish time: %f \n while time %f",
                (time_end_publish - time_begin_publish).seconds(),
                (time_end_publish - time_begin_while).seconds()
            );
        }
    }

private:
    cv::Mat cv_image;
    cv::VideoCapture capture;
    sensor_msgs::msg::Image::SharedPtr image_msg;
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher publisher_;
    image_transport::CameraPublisher camera_publisher_;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    // sensor_msgs::msg::Image::UniquePtr msg;
};

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
