#pragma once

#include "armor_detector/armor.hpp"

namespace armor {
class NumberClassifier {
public:
    NumberClassifier(
        const std::string& model_path,
        const std::string& label_path,
        const float confidence_threshold,
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
     * @brief 对装甲板进行分类，结果存入 armor.classfication_result
     *
     * @param armors 装甲板的容器
     */
    void Classify(std::vector<Armor>& armors);

    /**
     * @brief 更新忽略的类别
     *
     * @param ignore_classes 忽略的类别
     */
    void UpdateIgnoreClasses(const std::vector<std::string>& ignore_classes);

private:
    float confidence_threshold_;              // 数字分类置信度阈值
    cv::dnn::Net net_;                        // 数字分类网络
    std::vector<std::string> class_names_;    // 类别名字
    std::vector<std::string> ignore_classes_; // 忽略的类别
};
} // namespace armor
