#include <tf2/LinearMath/Matrix3x3.h>

#include "armor_detector/armor.hpp"
#include "armor_detector/pnp_solver.hpp"

namespace armor {

PnPSolver::PnPSolver(
    const std::vector<double>& camera_matrix,
    const std::vector<double>& distortion_coefficients
):
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone()),
    distortion_coefficients_(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_coefficients.data())).clone()) {}

void PnPSolver::CalculatePose(Armor& armor) {
    this->SolvePnP(armor);

    armor.pose.position.x = tvec_.at<double>(0);
    armor.pose.position.y = tvec_.at<double>(1);
    armor.pose.position.z = tvec_.at<double>(2);
    // 旋转向量 to 旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec_, rotation_matrix);
    // tf2 旋转矩阵
    tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix.at<double>(0, 0),
        rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0),
        rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2)
    );
    // 旋转矩阵 to 四元数
    tf2::Quaternion tf2_q;
    tf2_rotation_matrix.getRotation(tf2_q);
    armor.pose.orientation = tf2::toMsg(tf2_q);

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
    float cx = camera_matrix_.at<double>(0, 2); // 光学中心 x
    float cy = camera_matrix_.at<double>(1, 2); // 光学中心 y
    return cv::norm(armor_center - cv::Point2f(cx, cy));
}
} // namespace armor