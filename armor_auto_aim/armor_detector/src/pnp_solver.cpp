#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace armor {
PnPSolver::PnPSolver(
    const bool iterative,
    const std::array<double, 9>& camera_matrix,
    const std::vector<double>& dist_coeffs
):
    iterative_(iterative),
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone()),
    dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double*>(dist_coeffs.data())).clone()) {
    // Unit: m
    constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up
    small_armor_points_.emplace_back(0, small_half_y, -small_half_z);
    small_armor_points_.emplace_back(0, small_half_y, small_half_z);
    small_armor_points_.emplace_back(0, -small_half_y, small_half_z);
    small_armor_points_.emplace_back(0, -small_half_y, -small_half_z);

    large_armor_points_.emplace_back(0, large_half_y, -large_half_z);
    large_armor_points_.emplace_back(0, large_half_y, large_half_z);
    large_armor_points_.emplace_back(0, -large_half_y, large_half_z);
    large_armor_points_.emplace_back(0, -large_half_y, -large_half_z);
}

bool PnPSolver::SolvePnP(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec) {
    // 装甲板四个点在图像中的坐标
    std::vector<cv::Point2f> image_armor_points;
    image_armor_points.emplace_back(armor.left_light.bottom);
    image_armor_points.emplace_back(armor.left_light.top);
    image_armor_points.emplace_back(armor.right_light.top);
    image_armor_points.emplace_back(armor.right_light.bottom);

    // 装甲板四个点在三维坐标系中的坐标
    auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
    // PnP
    bool is_success = cv::solvePnP(
        object_points,
        image_armor_points,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );
    if (this->iterative_) {
        // 迭代第二次
        is_success = cv::solvePnP(
            object_points,
            image_armor_points,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            true,
            cv::SOLVEPNP_IPPE
        );
    }
    return is_success;
}

float PnPSolver::CalculateDistanceToCenter(const cv::Point2f& armor_center) {
    float cx = camera_matrix_.at<double>(0, 2); // 光学中心 x
    float cy = camera_matrix_.at<double>(1, 2); // 光学中心 y
    return cv::norm(armor_center - cv::Point2f(cx, cy));
}

} // namespace armor
