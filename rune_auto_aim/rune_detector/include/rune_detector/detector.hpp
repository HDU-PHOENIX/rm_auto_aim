#ifndef RUNE_DETECTOR__DETECTOR_HPP_
#define RUNE_DETECTOR__DETECTOR_HPP_

#include "nn.h"
#include "rune_type.hpp"
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <opencv4/opencv2/core/types.hpp>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "auto_aim_interfaces/msg/debug_runes.hpp"
#include "rune_detector/number_classifier.hpp"
#include "rune_detector/rune.hpp"

namespace rune {
class RuneDetector {
public:
    // 神符参数
    struct Data {
        cv::Point2f symbol; // 神符中心
        cv::Point2f armor_center; // 装甲板中心
        std::vector<cv::Point2f> vertices; // 灯条中心
    };

    RuneDetector();

    /**
     * @brief 检测神符 神经网络
     *
     * @param input 需要检测的图片
     * @return std::vector<Rune> 检测到的神符
     */
    void Detect(const cv::Mat& input);

private:
    NeuralNetwork yolo;
    bool Yolo();
    double confidence;
    std::vector<NeuralNetwork::RuneObject> objects;
    Data* data;
    // std::vector<Rune> runes_;  // 存放神符的容器
};

} // namespace rune

#endif // RUNE_DETECTOR__DETECTOR_HPP_
