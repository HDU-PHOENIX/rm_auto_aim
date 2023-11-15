#include <Eigen/Dense>
#include <array>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vector>
namespace camerainfo {
class CameraInfoNode: public rclcpp::Node {
public:
    explicit CameraInfoNode(const rclcpp::NodeOptions& options);

private:
    sensor_msgs::msg::CameraInfo camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    // ����ڲξ���
    // cv::Mat I_MAT;
    // // ����������
    // cv::Mat D_MAT;
    std::vector<double> camera_matrix_; //����ڲ�
    std::vector<double> distortion_coefficients_; //����������
};
} // namespace camerainfo