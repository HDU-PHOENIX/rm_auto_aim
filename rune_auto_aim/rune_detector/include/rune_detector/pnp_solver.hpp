#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "rune_detector/rune.hpp"

namespace rune {
class PnPSolver {
public:
    PnPSolver(const std::array<double, 9> &camera_matrix,
                        const std::vector<double> &distortion_coefficients);

    /**
     * @brief PnP 解算，获取 3D 位姿
     *
     * @param rune 装甲板
     * @param rvec TODO: description 
     * @param tvec TODO: description
     * @return bool 
     */
    bool SolvePnP(std::vector<cv::Point2f>, cv::Mat &rvec, cv::Mat &tvec);

    /**
     * @brief 计算装甲板中心到图像中心的距离
     *
     * @param image_point 装甲板中心点
     * @return float 距离 
     */
    // Calculate the distance between rune center and image center
    float CalculateDistanceToCenter(const cv::Point2f &image_point);

private:
    cv::Mat camera_matrix_;  // 相机参数矩阵 ？
    cv::Mat dist_coeffs_;    // 距离系数 ？

    // 神符的顶点的三维坐标
    //std::vector<cv::Point3f> rune_points_;
};

} // namespace rm_auto_aim

#endif // ARMOR_DETECTOR__PNP_SOLVER_HPP_
