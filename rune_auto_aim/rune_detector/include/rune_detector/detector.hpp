#ifndef RUNE_DETECTOR__DETECTOR_HPP_
#define RUNE_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "rune_detector/rune.hpp"
#include "rune_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_runes.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim {
class Detector {
public:
    // 神符参数
    struct RuneParams {
        
    };

    Detector();

    /**
     * @brief 检测神符 通过灯条检测和灯条匹配
     *
     * @param input 需要检测的图片
     * @return std::vector<Rune> 检测到的神符
     */
    std::vector<Rune> Detect(const cv::Mat &input);

    void DrawResults(cv::Mat &img);

    int detect_color;           // 识别到的颜色
    RuneParams rune_params;   // 神符参数

    // Debug 信息
    cv::Mat binary_img;
    auto_aim_interfaces::msg::DebugLights debug_lights;
    auto_aim_interfaces::msg::DebugRunes debug_runes;

private:
    std::vector<Rune> runes_;  // 存放神符的容器
};

} // namespace rm_auto_aim

#endif // RUNE_DETECTOR__DETECTOR_HPP_
