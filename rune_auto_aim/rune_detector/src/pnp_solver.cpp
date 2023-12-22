#include "rune_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

namespace rune {
PnPSolver::PnPSolver(
    const std::array<double, 9>& camera_matrix,
    const std::vector<double>& dist_coeffs
):
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone()),
    dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double*>(dist_coeffs.data())).clone()) {
}

std::vector<cv::Point3d>
PnPSolver::GeneratePw(double outerwidth, double insidewidth, double height) {
    return {
        { -1 * outerwidth / 2, height / 2, 0 },
        { -1 * insidewidth / 2, -1 * height / 2, 0 },
        { insidewidth / 2, -1 * height / 2, 0 },
        { outerwidth / 2, height / 2, 0 },
    };
}

/**
         * @brief 使用PnP根据像素坐标获取平移向量并转换成Eigen::Vector3d
         *
         * @param rune 符叶像素坐标
         * @return tvec 平移向量，相机坐标系下的坐标
         */
bool PnPSolver::SolvePnP(std::vector<cv::Point2d>& rune, cv::Mat& rvec, cv::Mat& tvec, bool iterate) {
    // Solve pnp
    bool ret = cv::solvePnP(
        GeneratePw(RUNE_PNP_OUTER_LIGHTBAR_WIDTH, RUNE_PNP_INSIDE_LIGHTBAR_WIDTH, RUNE_PNP_RADIUS),
        rune,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (iterate) {
        ret = cv::solvePnP(
        GeneratePw(RUNE_PNP_OUTER_LIGHTBAR_WIDTH, RUNE_PNP_INSIDE_LIGHTBAR_WIDTH, RUNE_PNP_RADIUS),
        rune,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        true,
        cv::SOLVEPNP_IPPE
    );
    }
    return ret;
}

float PnPSolver::CalculateDistanceToCenter(const cv::Point2f& image_point) {
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

} // namespace rune
