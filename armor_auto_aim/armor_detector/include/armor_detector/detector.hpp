#pragma once

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"

namespace armor {

struct DetectorParam {
    // 参数顺序参考 emum Color, 0:RED 1:BLUE
    int binary_threshold[2], light_contour_threshold[2];
};

class Detector {
public:
    Detector(
        DetectorParam detector_param,
        std::string model_path,
        std::string label_path,
        float confidence_threshold,
        const std::vector<double>& camera_matrix,
        const std::vector<double>& distortion_coefficients,
        Color enemy_color = Color::BLUE,
        std::vector<std::string> ignore_classes = { "negative" },
        cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U)
    );

    Detector(
        DetectorParam detector_param,
        std::unique_ptr<NumberClassifier> classifier,
        std::unique_ptr<PnPSolver> pnp_solver,
        Color enemy_color = Color::BLUE,
        cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U)
    );

    /**
     * @brief 对输入图片进行装甲板检测
     * @param input 输入图片
     * @return 装甲板的集合
     */
    std::vector<Armor> DetectArmor(const cv::Mat& input);

    /**
     * @brief 在输入图片上绘制装甲板和灯条
     * @param input 输入图片
     */
    void DrawResult(const cv::Mat& input) const;

    inline std::vector<Armor> GetDebugArmors() const {
        return debug_armors_;
    }
    inline std::vector<Light> GetDebugLights() const {
        return debug_lights_;
    }
    inline cv::Mat GetBinaryImage() const {
        return light_contour_binary_image_;
    }

    /**
     * @brief 获取所有数字的图像
     * @return 所有数字的图像
     */
    cv::Mat GetAllNumbersImage() const;

    /**
     * @brief 更新忽略的类别
     * @param ignore_classes 忽略的类别
     */
    inline void UpdateIgnoreClasses(const std::vector<std::string>& ignore_classes) {
        classifier_->UpdateIgnoreClasses(ignore_classes);
    }

    /**
     * @brief 更新敌方颜色
     * @param enemy_color 敌方颜色
     */
    inline void UpdateEnemyColor(const Color& enemy_color) {
        enemy_color_ = enemy_color;
    }

    inline void UpdateDetectorParam(const DetectorParam& detector_param) {
        detector_param_ = detector_param;
    }

private:
    /**
     * @brief 对输入图片进行预处理
     * @param input 输入图片
     * @return 预处理后的图片
     */
    cv::Mat PreprocessImage(const cv::Mat& input) const;

    /**
     * @brief 从预处理后的图片中检测灯条
     * @param input 预处理后的图片
     * @return 灯条的集合
     */
    std::vector<Light> DetectLight(const cv::Mat& input);

    /**
     * @brief 从灯条集合中筛选出装甲板
     * @param lights 灯条集合
     * @return 装甲板集合
     */
    std::vector<Armor> FilterArmor(const cv::Mat& input, const std::vector<Light>& lights);

    /**
     * @brief 从灯条轮廓构成灯条
     * @param light_contour 灯条轮廓
     * @return 灯条
     */
    Light FormLight(const std::vector<cv::Point>& light_contour);

    /**
     * @brief 通过两个灯条判断是否能够组成装甲板
     * @param left_light 左灯条
     * @param right_light 右灯条
     * @return 组成的装甲板
     */
    Armor FormArmor(const cv::Mat& input, const Light& left_light, const Light& right_light);

    /**
     * @brief 判断两个灯条之间是否包含其他灯条
     * @param light1 第一个灯条
     * @param light2 第二个灯条
     *
     * @return 是否包含其他灯条
     */
    bool ContainLight(const Light& light1, const Light& light2) const;

    DetectorParam detector_param_;
    cv::Mat preprocessed_image_;         // 预处理后的图片
    cv::Mat channels_[3];                // 通道相减模式下的三通道图
    cv::Mat color_mask_;                 // 敌方颜色通道 - 己方颜色通道后的图
    cv::Mat light_contour_binary_image_; // 灯条轮廓为白的二值图
    Color enemy_color_;                  // 敌方颜色
    cv::Mat kernel_;                     // 膨胀核

    std::vector<Light> lights_;       // 合法灯条集合
    std::vector<Armor> armors_;       // 合法装甲板集合
    std::vector<Light> debug_lights_; // 所有灯条集合
    std::vector<Armor> debug_armors_; // 所有装甲板集合

    std::unique_ptr<NumberClassifier> classifier_;
    std::unique_ptr<PnPSolver> pnp_solver_;
};

} // namespace armor
