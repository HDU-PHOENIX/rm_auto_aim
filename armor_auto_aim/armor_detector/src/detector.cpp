// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace armor {
Detector::Detector(
    const int& bin_thres,
    const int& color,
    const LightParams& light_params,
    const ArmorParams& armor_params
):
    binary_thres(bin_thres),
    detect_color(color),
    light_params(light_params),
    armor_params(armor_params) {}

std::vector<Armor> Detector::Detect(const cv::Mat& input) {
    binary_img = PreprocessImage(input); // 二值化图像
    lights_ = FindLights(input, binary_img); // 检测灯条
    armors_ = MatchLights(lights_); // 匹配灯条

    // 若图像含有装甲板，进行数字提取和分类
    if (!armors_.empty()) {
        classifier->ExtractNumbers(input, armors_);
        classifier->Classify(armors_);
    }

    return armors_;
}

cv::Mat Detector::PreprocessImage(const cv::Mat& rgb_img) {
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

    return binary_img;
}

std::vector<Light> Detector::FindLights(const cv::Mat& rbg_img, const cv::Mat& binary_img) {
    using std::vector;

    // 寻找灯条轮廓
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<Light> lights;
    this->debug_lights.data.clear();

    // 遍历轮廓，寻找灯条并判断颜色
    for (const auto& contour: contours) {
        // TODO: 为什么要大于等于 5
        if (contour.size() < 5) {
            continue;
        }

        // 构造最小外接矩形，并转换为 Light 类
        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        if (IsLight(light)) {
            // 获取包含灯条的最小直立矩形
            auto rect = light.boundingRect();

            // 防止越界
            if (0 <= rect.x && 0 <= rect.width && 0 <= rect.y && 0 <= rect.height
                && rect.x + rect.width <= rbg_img.cols && rect.y + rect.height <= rbg_img.rows)
            {
                int sum_r = 0, sum_b = 0;
                auto roi = rbg_img(rect);
                // 遍历 ROI，计算灯条颜色
                for (int i = 0; i < roi.rows; i++) {
                    for (int j = 0; j < roi.cols; j++) {
                        // TODO: 这个判断是否多余？
                        if (cv::pointPolygonTest(
                                contour,
                                cv::Point2f(j + rect.x, i + rect.y),
                                false
                            )
                            >= 0) {
                            sum_r += roi.at<cv::Vec3b>(i, j)[0];
                            sum_b += roi.at<cv::Vec3b>(i, j)[2];
                        }
                    }
                }
                // 直接通过像素值加和判断颜色
                light.color = sum_r > sum_b ? RED : BLUE;
                lights.emplace_back(light);
            }
        }
    }

    return lights;
}

bool Detector::IsLight(const Light& light) {
    // 计算灯条长宽比和倾斜角度判断是否为灯条

    // The ratio of light (short side / long side)
    float ratio = light.width / light.length;
    bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;

    bool angle_ok = light.tilt_angle < light_params.max_angle;

    bool is_light = ratio_ok && angle_ok;

    // debug information
    auto_aim_interfaces::msg::DebugLight light_data;
    light_data.center_x = light.center.x;
    light_data.ratio = ratio;
    light_data.angle = light.tilt_angle;
    light_data.is_light = is_light;
    this->debug_lights.data.emplace_back(light_data);

    return is_light;
}

std::vector<Armor> Detector::MatchLights(const std::vector<Light>& lights) {
    std::vector<Armor> armors;
    this->debug_armors.data.clear();

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_1->color != detect_color || light_2->color != detect_color) {
                continue;
            }

            if (ContainLight(*light_1, *light_2, lights)) {
                continue;
            }

            auto type = IsArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::ContainLight(
    const Light& light_1,
    const Light& light_2,
    const std::vector<Light>& lights
) {
    auto points =
        std::vector<cv::Point2f> { light_1.top, light_1.bottom, light_2.top, light_2.bottom };
    auto bounding_rect = cv::boundingRect(points);

    for (const auto& test_light: lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center) {
            continue;
        }

        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom)
            || bounding_rect.contains(test_light.center))
        {
            return true;
        }
    }

    return false;
}

ArmorType Detector::IsArmor(const Light& light_1, const Light& light_2) {
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                               : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > armor_params.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (armor_params.min_small_center_distance <= center_distance
                               && armor_params.max_small_center_distance > center_distance)
        || (armor_params.min_large_center_distance <= center_distance
            && armor_params.max_large_center_distance > center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < armor_params.max_angle;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    ArmorType type;
    if (is_armor) {
        type = center_distance > armor_params.min_large_center_distance ? ArmorType::LARGE
                                                                        : ArmorType::SMALL;
    } else {
        type = ArmorType::INVALID;
    }

    // Fill in debug information
    auto_aim_interfaces::msg::DebugArmor armor_data;
    armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
    armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
    armor_data.light_ratio = light_length_ratio;
    armor_data.center_distance = center_distance;
    armor_data.angle = angle;
    this->debug_armors.data.emplace_back(armor_data);

    return type;
}

cv::Mat Detector::GetAllNumbersImage() {
    if (armors_.empty()) {
        return cv::Mat(cv::Size(20, 28), CV_8UC1);
    } else {
        std::vector<cv::Mat> number_imgs;
        number_imgs.reserve(armors_.size());
        for (auto& armor: armors_) {
            number_imgs.emplace_back(armor.number_img);
        }
        cv::Mat all_num_img;
        cv::vconcat(number_imgs, all_num_img);
        return all_num_img;
    }
}

void Detector::DrawResults(cv::Mat& img) {
    // Draw Lights
    for (const auto& light: lights_) {
        cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
        auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
        cv::line(img, light.top, light.bottom, line_color, 1);
    }

    // Draw armors
    for (const auto& armor: armors_) {
        cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
        cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
    }

    // Show numbers and confidence
    for (const auto& armor: armors_) {
        cv::putText(
            img,
            armor.classfication_result,
            armor.left_light.top,
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 255, 255),
            2
        );
    }
}

} // namespace armor
