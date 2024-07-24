#include <fstream>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"

namespace armor {
NumberClassifier::NumberClassifier(
    const std::string& model_path,
    const std::string& label_path,
    const float confidence_threshold,
    const std::vector<std::string>& ignore_classes
):
    confidence_threshold_(confidence_threshold),
    ignore_classes_(ignore_classes) {
    net_ = cv::dnn::readNetFromONNX(model_path);
    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line);
    }
}

void NumberClassifier::ExtractNumbers(const cv::Mat& src, Armor& armor) {
    // 透视变换
    cv::Point2f lights_vertices[4] = {
        armor.left_light.bottom,
        armor.left_light.top,
        armor.right_light.top,
        armor.right_light.bottom
    };

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // 获取数字 ROI
    number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // 二值化
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_image = number_image;
}

bool NumberClassifier::Classify(const cv::Mat& src, Armor& armor) {
    ExtractNumbers(src, armor);

    cv::Mat image = armor.number_image.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.classification_confidence = confidence;
    armor.number = class_names_[label_id];

    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.classification_confidence * 100.0 << "%";
    armor.classification_result = result_ss.str();

    // 装甲板置信度过低
    if (armor.classification_confidence < confidence_threshold_) {
        armor.classification_result = "confidence too low";
        return false;
    }

    // 忽略的装甲板类型
    for (const auto& ignore_class: ignore_classes_) {
        if (armor.number == ignore_class) {
            armor.classification_result = "ignore class";
            return false;
        }
    }

    // 错误匹配的类型
    bool mismatch_armor_type = false;
    if (armor.type == ArmorType::LARGE) {
        mismatch_armor_type = armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
    } else if (armor.type == ArmorType::SMALL) {
        mismatch_armor_type = armor.number == "1" || armor.number == "base";
    }
    if (mismatch_armor_type) {
        armor.classification_result = "mismatch armor type";
        return false;
    }

    armor.classification_result = "success";
    return true;
}

void NumberClassifier::UpdateIgnoreClasses(const std::vector<std::string>& ignore_classes) {
    ignore_classes_ = ignore_classes;
}

} // namespace armor