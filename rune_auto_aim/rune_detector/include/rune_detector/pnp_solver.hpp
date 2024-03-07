#ifndef RUNE_DETECTOR__PNP_SOLVER_HPP_
#define RUNE_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <opencv4/opencv2/core/types.hpp>
#include <vector>

#include "rune_detector/sizes.hpp"

namespace rune {
class PnPSolver {
public:
    explicit PnPSolver(
        const std::vector<double>& camera_matrix,
        const std::vector<double>& distortion_coefficients
    );

    /**
     * @brief PnP 解算，获取 3D 位姿
     *
     * @param rune 装甲板
     * @param rvec TODO: description 
     * @param tvec TODO: description
     * @return bool 
     */
    bool SolvePnP(std::vector<cv::Point2d>& rune, cv::Mat& rvec, cv::Mat& tvec, bool iterate);

    /**
     * @brief 计算装甲板中心到图像中心的距离
     *
     * @param image_point 装甲板中心点
     * @return float 距离 
     */
    // Calculate the distance between rune center and image center
    float CalculateDistanceToCenter(const cv::Point2f& image_point);

    static std::vector<cv::Point3d>
    GeneratePw(double outerwidth, double insidewidth, double height);

private:
    cv::Mat camera_matrix_;           // 相机参数矩阵 相机内参
    cv::Mat distortion_coefficients_; // 相机畸变参数矩阵

    // 神符的顶点的三维坐标
    //std::vector<cv::Point3f> rune_points_;
};

} // namespace rune

#endif
