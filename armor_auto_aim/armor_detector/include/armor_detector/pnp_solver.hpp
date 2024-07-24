#include "armor_detector/armor.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace armor {

class PnPSolver {
public:
    explicit PnPSolver(
        const std::vector<double>& camera_matrix,
        const std::vector<double>& distortion_coefficients
    );

    /**
     * @brief 通过 PnP 计算装甲板位姿势
     * @param armor
     */
    void CalculatePose(Armor& armor);

    /**
     * @brief 返回相机中心坐标
     *
     * @return Point2f 相机中心二维坐标
     */
    inline cv::Point2f GetCameraCenter() const {
        return camera_center_;
    }

private:
    /**
     * @brief 计算装甲板中心到图像中心的距离
     *
     * @param image_point 装甲板中心点
     *
     * @return float 距离 
     */
    inline float CalculateDistanceToCenter(const cv::Point2f& armor_center) const {
        return static_cast<float>(cv::norm(armor_center - camera_center_));
    };

    /**
     * @brief PnP 求解
     * 
     * @param armor 
     */
    void SolvePnP(const Armor& armor);

    cv::Point2f camera_center_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    cv::Mat rvec_;
    cv::Mat tvec_;
};

} // namespace armor
