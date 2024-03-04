#include <geometry_msgs/msg/pose.hpp>

#include "armor_detector/armor.hpp"

namespace armor {

class PnPSolver {
public:
    explicit PnPSolver(
        const std::vector<double>& camera_matrix,
        const std::vector<double>& distortion_coefficients
    );

    void SolvePnP(const Armor& armor);

    /**
     * @brief 计算相机坐标系到装甲板坐标系的旋转矩阵
     * @param pose
     */
    void CalculatePose(Armor& armor);

    /**
     * @brief 计算装甲板中心到图像中心的距离
     *
     * @param image_point 装甲板中心点
     *
     * @return float 距离 
     */
    float CalculateDistanceToCenter(const cv::Point2f& armor_center);

    /**
     * @brief 返回相机中心坐标
     *
     * @return Point2f 相机中心二维坐标
     */
    cv::Point2f GetCameraCenter();

private:
    cv::Point2f camera_center_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    cv::Mat rvec_;
    cv::Mat tvec_;
};

} // namespace armor
