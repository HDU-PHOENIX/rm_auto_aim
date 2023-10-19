#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

// OpenCV
#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

#define RED 0
#define BLUE 1

namespace rm_auto_aim {

// 装甲板类型：小、大、无效
enum class RuneType { SMALL, LARGE, INVALID };
// const std::string ARMOR_TYPE_STR[3] = { "small", "large", "invalid" };

struct Rune {
    Rune() = default;

    float confidence;     // 装甲板数字分类置信度
    std::string classfication_result; // 装甲板数字分类结果
};

} // namespace rm_auto_aim

#endif // ARMOR_DETECTOR__ARMOR_HPP_
