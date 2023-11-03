#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

// OpenCV
#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

#define RED  0
#define BLUE 1

namespace armor {

// 装甲板类型：小、大、无效
enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = { "small", "large", "invalid" };

struct Light: public cv::RotatedRect {
    Light() = default;
    explicit Light(cv::RotatedRect box): cv::RotatedRect(box) {
        cv::Point2f p[4];
        box.points(p);

        // 对四个角点从上到下排序，保证前两个为 top，后两个为 bottom
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        // 计算灯条长度、宽度、倾斜角度
        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    // 灯条颜色 RED or BLUE
    int color;
    // 灯条上下边框中点坐标
    cv::Point2f top, bottom;
    // 灯条长度
    double length;
    // 灯条宽度
    double width;
    // 灯条倾斜角度，相对于垂直面
    float tilt_angle;
};

struct Armor {
    Armor() = default;
    Armor(const Light& l1, const Light& l2) {
        if (l1.center.x < l2.center.x) {
            left_light = l1, right_light = l2;
        } else {
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
    }

    Light left_light, right_light; // 左右灯条
    cv::Point2f center;            // 装甲板中心
    ArmorType type;                // 装甲板类型

    cv::Mat number_img;               // 装甲板数字图像
    std::string number;               // 装甲板数字
    float confidence;                 // 装甲板数字分类置信度
    std::string classfication_result; // 装甲板数字分类结果
};

} // namespace armor

#endif // ARMOR_DETECTOR__ARMOR_HPP_
