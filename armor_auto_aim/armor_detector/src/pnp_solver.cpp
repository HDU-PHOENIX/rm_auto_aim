#include "armor_detector/pnp_solver.hpp"
#include "armor_detector/armor.hpp"

namespace armor {

PnPSolver::PnPSolver(
    const std::vector<double>& camera_matrix,
    const std::vector<double>& distortion_coefficients
):
    camera_center_(camera_matrix.at(2), camera_matrix.at(5)),
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone()),
    distortion_coefficients_(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_coefficients.data())).clone()) {}

void PnPSolver::CalculatePose(Armor& armor) {
    this->SolvePnP(armor);

    armor.position.x = tvec_.at<double>(0);
    armor.position.y = tvec_.at<double>(1);
    armor.position.z = tvec_.at<double>(2);

    armor.distance_to_image_center = cv::norm(
        armor.center - cv::Point2f(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2))
    );

    armor.distance_to_center = CalculateDistanceToCenter(armor.center);
}

void PnPSolver::SolvePnP(const Armor& armor) {
    std::vector<cv::Point2f> image_armor_points;
    image_armor_points.emplace_back(armor.left_light.bottom);
    image_armor_points.emplace_back(armor.left_light.top);
    image_armor_points.emplace_back(armor.right_light.top);
    image_armor_points.emplace_back(armor.right_light.bottom);

    // 装甲板四个点在三维坐标系中的坐标
    auto object_points = armor.type == ArmorType::SMALL ? SMALL_ARMOR_POINTS : LARGE_ARMOR_POINTS;

    cv::solvePnP(
        object_points,
        image_armor_points,
        camera_matrix_,
        distortion_coefficients_,
        rvec_,
        tvec_,
        false,
        cv::SOLVEPNP_IPPE
    );
}

float PnPSolver::CalculateDistanceToCenter(const cv::Point2f& armor_center) {
    return cv::norm(armor_center - camera_center_);
}

cv::Point2f PnPSolver::GetCameraCenter() {
    return camera_center_;
}

} // namespace armor