#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace armor {
class NumberClassifier {
public:
    NumberClassifier(
        const std::string& model_path,
        const std::string& label_path,
        const double threshold,
        const std::vector<std::string>& ignore_classes = {}
    );

    /**
     * @brief 提取数字的图像存入 armor.number_img
     *
     * @param src 原始图像
     * @param armors 包含所有装甲板的容器
     */
    void ExtractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

    /**
     * @brief 对装甲板进行分类，结果存入 armor.classification_result
     *
     * @param armors 装甲板的容器
     */
    void Classify(std::vector<Armor>& armors);

    // 数字分类置信度阈值
    double threshold;

private:
    cv::dnn::Net net_;                        // 数字分类网络
    std::vector<std::string> class_names_;    // 类别名字
    std::vector<std::string> ignore_classes_; // 忽略的类别
};
} // namespace armor

#endif // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
