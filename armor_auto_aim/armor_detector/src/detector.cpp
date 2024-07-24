#include "armor_detector/detector.hpp"

namespace armor {
Detector::Detector(
    DetectorParam detector_param,
    std::string model_path,
    std::string label_path,
    float confidence_threshold,
    const std::vector<double>& camera_matrix,
    const std::vector<double>& distortion_coefficients,
    Color enemy_color,
    std::vector<std::string> ignore_classes,
    cv::Mat kernel
):
    detector_param_(detector_param),
    enemy_color_(enemy_color),
    kernel_(kernel) {
    this->classifier_ = std::make_unique<NumberClassifier>(model_path, label_path, confidence_threshold, ignore_classes);
    this->pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, distortion_coefficients);
}

Detector::Detector(
    DetectorParam detector_param,
    std::unique_ptr<NumberClassifier> classifier,
    std::unique_ptr<PnPSolver> pnp_solver,
    Color enemy_color,
    cv::Mat kernel
):
    detector_param_(detector_param),
    enemy_color_(enemy_color),
    kernel_(kernel),
    classifier_(std::move(classifier)),
    pnp_solver_(std::move(pnp_solver)) {}

std::vector<Armor> Detector::DetectArmor(const cv::Mat& input) {
    preprocessed_image_ = PreprocessImage(input);
    lights_ = DetectLight(input);
    armors_ = FilterArmor(input, lights_);

    if (!armors_.empty()) {
        for (auto& armor: armors_) {
            pnp_solver_->CalculatePose(armor);
        }
    }

    return armors_;
}

cv::Mat Detector::PreprocessImage(const cv::Mat& input) const {
    cv::Mat gray;
    cv::Mat binary;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(
        gray,
        binary,
        detector_param_.binary_threshold[static_cast<int>(enemy_color_)],
        255,
        cv::THRESH_BINARY
    );
    return binary;
}

std::vector<Light> Detector::DetectLight(const cv::Mat& input) {
    std::vector<Light> lights;
    debug_lights_.clear();

    // 敌方颜色通道 - 己方颜色通道
    cv::split(input, channels_);
    if (enemy_color_ == Color::RED) {
        cv::subtract(channels_[2], channels_[0], color_mask_);
    } else {
        cv::subtract(channels_[0], channels_[2], color_mask_);
    }
    cv::threshold(
        color_mask_,
        light_contour_binary_image_,
        detector_param_.light_contour_threshold[static_cast<int>(enemy_color_)],
        255,
        cv::THRESH_BINARY
    );
    // 膨胀，使灯条更加连续
    cv::dilate(light_contour_binary_image_, light_contour_binary_image_, kernel_);
    // 与预处理图像取交集，获取又亮又符合颜色的区域
    cv::bitwise_and(preprocessed_image_, light_contour_binary_image_, light_contour_binary_image_);

    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(this->light_contour_binary_image_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour: contours) {
        if (contour.size() < 4) {
            continue;
        }

        Light light = FormLight(contour);
        if (light.valid == true) {
            lights.push_back(light);
        }
    }

    return lights;
}

bool Detector::ContainLight(const Light& light1, const Light& light2) const {
    return std::ranges::any_of(lights_, [&](const Light& light) {
        return (light.center.y > light1.center.y && light.center.y < light2.center.y)
            && (light.center.x > light1.center.x && light.center.x < light2.center.x);
    });
}

std::vector<Armor> Detector::FilterArmor(const cv::Mat& input, const std::vector<Light>& lights) {
    std::vector<Armor> armors;
    armors.reserve(lights.size() / 2);
    debug_armors_.clear();

    // 两两配对遍历灯条进行装甲板匹配
    for (const auto& left_light: lights) {
        for (const auto& right_light: lights) {
            if (left_light.center.x >= right_light.center.x) {
                continue;
            }

            Armor armor = FormArmor(input, left_light, right_light);
            if (armor.type != ArmorType::INVALID) {
                armors.push_back(armor);
            }
        }
    }

    return armors;
}

Light Detector::FormLight(const std::vector<cv::Point>& light_contour) {
    Light light(cv::minAreaRect(light_contour));
    light.valid = (light.length > light.width * 3) && (std::abs(light.tilt_angle) < 25);

    debug_lights_.push_back(light);
    return light;
}

Armor Detector::FormArmor(const cv::Mat& input, const Light& left_light, const Light& right_light) {
    Armor armor(left_light, right_light);
    bool light_height_ratio_valid = armor.light_height_ratio < 1.2 && armor.light_height_ratio > 0.8;
    bool light_angle_diff_valid = armor.light_angle_diff < 10;
    bool angle_valid = armor.angle < 30;
    bool light_center_distance_valid = (armor.light_center_distance > 0.8 && armor.light_center_distance < 3.2)
        || (armor.light_center_distance > 3.2 && armor.light_center_distance < 5.5);

    using enum armor::ArmorType;
    if (light_height_ratio_valid && light_angle_diff_valid && angle_valid && light_center_distance_valid
        && !ContainLight(left_light, right_light)) {
        armor.type = (armor.light_center_distance > 3.2) ? LARGE : SMALL;

        if (!classifier_->Classify(input, armor)) {
            armor.type = INVALID;
        }
    } else {
        armor.type = INVALID;
    }

    debug_armors_.push_back(armor);
    return armor;
}

cv::Mat Detector::GetAllNumbersImage() const {
    if (armors_.empty()) {
        return cv::Mat(cv::Size(20, 28), CV_8UC1);
    } else {
        std::vector<cv::Mat> number_imgs;
        number_imgs.reserve(armors_.size());
        for (const auto& armor: armors_) {
            number_imgs.emplace_back(armor.number_image);
        }
        cv::Mat all_num_img;
        cv::vconcat(number_imgs, all_num_img);
        return all_num_img;
    }
}

void Detector::DrawResult(const cv::Mat& input) const {
    cv::circle(input, pnp_solver_->GetCameraCenter(), 5, cv::Scalar(255, 0, 0), 1);

    for (const auto& light: lights_) {
        cv::line(input, light.top, light.bottom, cv::Scalar(0, 255, 0), 1);
    }

    for (const auto& armor: armors_) {
        cv::line(input, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 1);
        cv::line(input, armor.left_light.bottom, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
        cv::circle(input, armor.center, 3, cv::Scalar(0, 255, 0), 1);
        cv::putText(input, armor.number, armor.center, cv::FONT_HERSHEY_SIMPLEX, 2.5, cv::Scalar(255, 0, 0), 1);
    }
}

} // namespace armor
