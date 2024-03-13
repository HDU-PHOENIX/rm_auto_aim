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
     * @brief 对装甲板进行分类，结果存入 armor.classfication_result
     *
     * @param armors 装甲板的容器
     * @return 是否分类成功（判断置信度 错误类型 忽略的类型）
     */
    bool Classify(const cv::Mat& src, Armor& armor);

    /**
     * @brief 更新忽略的类别
     *
     * @param ignore_classes 忽略的类别
     */
    void UpdateIgnoreClasses(const std::vector<std::string>& ignore_classes);

private:
    /**
     * @brief 提取数字的图像存入 armor.number_img
     *
     * @param src 原始图像
     * @param armors 包含所有装甲板的容器
     */
    void ExtractNumbers(const cv::Mat& src, Armor& armors);

    float confidence_threshold_;              // 数字分类置信度阈值
    cv::dnn::Net net_;                        // 数字分类网络
    std::vector<std::string> class_names_;    // 类别名字
    std::vector<std::string> ignore_classes_; // 忽略的类别

    const int light_length = 12;
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
    const cv::Size roi_size = cv::Size(20, 28);
};
} // namespace armor
