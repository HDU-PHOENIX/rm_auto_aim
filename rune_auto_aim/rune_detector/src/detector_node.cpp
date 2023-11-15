#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rune {
RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("rune_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
    // ��ʼ�����ʶ����
    detector_ = InitDetector();

    // ������Ƿ�����
    debug_ = this->declare_parameter("debug", false);
    if (debug_) {
        CreateDebugPublishers();
    }

    // ���������Ϣ������
    runes_pub_ = this->create_publisher<auto_aim_interfaces::msg::Runes>(
        "/detector/runes",
         rclcpp::SensorDataQoS());

    // ����ͼ������
    image_sub_ = image_transport::create_camera_subscription(
        this,
        "image_raw",
        std::bind(&RuneDetectorNode::ImageCallback, this, std::placeholders::_1),
        "raw",
        rmw_qos_profile_sensor_data
    );

    // ���������Ϣ������
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
            cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
            cam_info_sub_.reset();
        }
    );
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&RuneDetectorNode::ImageCallback, this, std::placeholders::_1)
    );
}

std::vector<Rune>
RuneDetectorNode::DetectRunes(const sensor_msgs::msg::Image::SharedPtr& img_msg){
    auto &&img = cv::Mat(img_msg->height, img_msg->width, CV_8UC3, img_msg->data.data());
    detector_->detect(img, objects_);//�����ʶ�����ŵ�objects_����
    //TODO: ʶ�𵽵������Ϣת��Ϊ�Զ���������Ϣ
}


void RuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg){
    RCLCPP_INFO(
        this->get_logger(),
        "timestamp: %d image address in detector %p",
        img_msg->header.stamp.nanosec,
        static_cast<void*>(const_cast<sensor_msgs::msg::Image*>(img_msg.get()))
    );
    auto runes = DetectRunes(img_msg);


}



std::unique_ptr<NeuralNetwork> RuneDetectorNode::InitDetector() {
    
    auto&& detector = std::make_unique<NeuralNetwork>();
    detector->init("path to yolox model");

    return detector;
}




} // namespace rune