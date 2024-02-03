#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace armor {
class Detector {
public:
    // 灯条参数
    struct LightParams {
        // 长宽比范围 width / height
        double min_ratio;
        double max_ratio;
        // 垂直角度
        double max_angle;
    };

    // 装甲板参数
    struct ArmorParams {
        // 左右灯条比例最小值
        double min_light_ratio;
        // 大小装甲板两灯条之间距离 / 灯条平均长度 的阈值
        double min_small_center_distance, max_small_center_distance, min_large_center_distance,
            max_large_center_distance;
        // 装甲板水平角度
        double max_angle;
    };

    Detector(
        const int& gray_thres,
        const int& contour_thres,
        const int& color,
        const LightParams& light_params,
        const ArmorParams& armor_params,
        const int& detect_mode
    );
    Detector(
        const int& bin_thres,
        const int& color,
        const LightParams& light_params,
        const ArmorParams& armor_params,
        const int& detect_mode
    );

    /**
     * @brief   检测装甲板
     * @details 处理图像 -> 寻找灯条 -> 匹配灯条 -> 提取数字 -> 分类
     *          PreprocessImage -> FindLights -> MatchLights -> ExtractNumbers -> Classify
     *
     * @param input 需要检测的图片
     * @return std::vector<Armor> 检测到的装甲板
     */
    std::vector<Armor> Detect(const cv::Mat& input);

    /**
     * @brief 对图像进行预处理
     *
     * @param input    需要预处理的图像
     * @return cv::Mat 预处理之后的二值化图像
     */
    cv::Mat PreprocessImage(const cv::Mat& input);

    /**
     * @brief 寻找灯条
     *
     * @param rbg_img    rgb 图像
     * @param binary_img 二值化图像
     * @return std::vector<Light> 寻找到的灯条的容器
     */
    std::vector<Light> FindLights(const cv::Mat& rbg_img, const cv::Mat& binary_img);

    /**
     * @brief 匹配灯条
     *
     * @param lights 灯条的容器
     * @return std::vector<Armor> 通过灯条匹配到的装甲板
     */
    std::vector<Armor> MatchLights(const std::vector<Light>& lights);

    /**
     * @brief 获取所有数字的图像
     * 
     * @return cv::Mat 
     */
    cv::Mat GetAllNumbersImage();
    /**
     * @brief 在图像上画出结果，显示数字和置信度
     * 
     * @param img 原始图像
     */
    void DrawResults(cv::Mat& img);

    int binary_thres;         // 二值化阈值
    int gray_thres;           // 灰度二值化阈值
    int contour_thres;        // 通道相减二值化阈值
    int enemy_color;          // 识别到的颜色
    LightParams light_params; // 灯条参数
    ArmorParams armor_params; // 装甲板参数
    int detect_mode;          // 检测模式 0: 默认 1: 通道相减二值化

    // 数字分类器
    std::unique_ptr<NumberClassifier> classifier;

    // 允许 detector_node 访问的用于调试的图像和信息
    cv::Mat gray_mask, color_mask, contour_mask, binary_img;
    auto_aim_interfaces::msg::DebugLights debug_lights;
    auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
    /**
     * @brief 判断是否是灯条
     *
     * @param possible_light 可能的灯条
     * @return bool
     */
    bool IsLight(const Light& possible_light);

    /**
     * @brief 两灯条间是否包含灯条
     *
     * @param light_1 灯条 1
     * @param light_2 灯条 2
     * @param lights  所有灯条的容器
     * @return bool
     */
    bool ContainLight(const Light& light_1, const Light& light_2, const std::vector<Light>& lights);

    /**
     * @brief 判度是否是装甲板
     *
     * @param light_1     装甲板灯条 1
     * @param light_2     装甲板灯条 2
     * @return ArmorType  装甲板类型
     */
    ArmorType IsArmor(const Light& light_1, const Light& light_2);

    std::vector<Light> lights_; // 存放灯条的容器
    std::vector<Armor> armors_; // 存放装甲板的容器
};

} // namespace armor

#endif // ARMOR_DETECTOR__DETECTOR_HPP_
