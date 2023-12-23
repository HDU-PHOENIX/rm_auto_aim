#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace armor {
class PnPSolver {
public:
    PnPSolver(
        const bool iterative,
        const std::array<double, 9>& camera_matrix,
        const std::vector<double>& distortion_coefficients
    );

    /**
     * @brief PnP 解算，获取 3D 位姿
     *
     * @param armor 装甲板
     * @param rvec 旋转向量
     * @param tvec 平移向量
     *
     * @return bool 是否解算成功        
     */
    bool SolvePnP(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec);

    /**
     * @brief 计算装甲板中心到图像中心的距离
     *
     * @param image_point 装甲板中心点
     *
     * @return float 距离 
     */
    float CalculateDistanceToCenter(const cv::Point2f& armor_center);

private:
    bool iterative_; // 是否迭代两次

    cv::Mat camera_matrix_; // 相机内参矩阵
    cv::Mat dist_coeffs_;   // 相机畸变系数

    // 大小装甲板宽高参数 Unit: mm
    static constexpr float SMALL_ARMOR_WIDTH = 135, SMALL_ARMOR_HEIGHT = 55,
                           LARGE_ARMOR_WIDTH = 225, LARGE_ARMOR_HEIGHT = 55;

    // 大装甲板的四个顶点的三维坐标
    std::vector<cv::Point3f> small_armor_points_;
    // 小装甲板的四个顶点的三维坐标
    std::vector<cv::Point3f> large_armor_points_;
};

} // namespace armor

#endif // ARMOR_DETECTOR__PNP_SOLVER_HPP_
