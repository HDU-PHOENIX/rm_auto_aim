#include "rune_detector/detector.hpp"
#include <opencv2/core/mat.hpp>

namespace rune {
RuneDetector::RuneDetector() {
    confidence = 0.7; //置信度
    yolo.init("path to yolo model"); //初始化神经网络
}

void RuneDetector::Detect(const cv::Mat& input) {
    // cv::Mat img = input.clone();
    // cv::Mat img = input;
    // cv
    Yolo(input);
}

bool RuneDetector::Yolo(cv::Mat& img) {
    if (!yolo.detect(img, objects)) {
        std::cout << "find no rune" << std::endl;
        //检测神符
    } else {
        //检测到神符
        RuneClass cls; //符叶枚举类对象
        bool flag1 = false, flag2 = false, flag3 = false;
        for (auto object: objects) {
            auto prob = object.prob;
            if (prob < confidence) {
                Log::Info("the confidence is :{}", prob);
                continue;
            }

            auto&& detect_center = (object.vertices[0] + object.vertices[1] + object.vertices[2]
                                    + object.vertices[3] + object.vertices[4])
                / 5;

            auto&& get_symbol = [](const cv::Point2f& lightbar_mid_point,
                                   const cv::Point2f& armor_center,
                                   const double& center_lightbar_ratio,
                                   const bool& flag) {
                //get_symbol是通过符叶的坐标来计算中心R标的位置
                if (flag == 0) {
                    //flag = 0使用装甲板中心和内灯条算出标识符位置
                    return (
                        (lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center
                    );

                } else if (flag == 1) {
                    //flag = 1使用装甲板中心和外灯条算出标识符位置
                    return (
                        -(lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center
                    );
                }
                return cv::Point2f(0, 0);
            };

            if (object.color == 0 && object.cls == 0) {
                cls = RuneClass::Blue;
                flag1 = true;
                data->symbol = detect_center;

            } else if (object.color == 1 && object.cls == 0) {
                cls = RuneClass::Red;
                flag1 = true;
                data->symbol = detect_center;

            } else if (object.color == 0 && object.cls == 1) {
                cls = RuneClass::BlueUnActivated;
                data->vertices.clear();

                data->vertices.push_back(object.vertices[1]);
                data->vertices.push_back(object.vertices[2]);
                data->vertices.push_back(object.vertices[4]);
                data->vertices.push_back(object.vertices[0]);
                auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
                auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
                auto&& armor = (tmp1 + tmp2) / 2;
                if (!flag1) //如果yolo没有检测到R标
                    data->symbol =
                        (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 3.5542, 0))
                        / 2;

                data->armor = armor;
                cv::circle(data->dst, data->armor, 4, Colors::Aqua, -1);
                cv::circle(data->dst, data->symbol, 4, Colors::Yellow, -1);
                flag1 = true;
                flag2 = true;

            } else if (object.color == 1 && object.cls == 1) {
                cls = RuneClass::RedUnActivated;
                data->vertices.clear();

                data->vertices.push_back(object.vertices[1]);
                data->vertices.push_back(object.vertices[2]);
                data->vertices.push_back(object.vertices[4]);
                data->vertices.push_back(object.vertices[0]);
                auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
                auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
                auto&& armor = (tmp1 + tmp2) / 2;
                if (!flag1) //如果yolo没有检测到R标
                    data->symbol =
                        (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 3.5542, 0))
                        / 2;

                data->armor = armor;
                cv::circle(data->dst, data->armor, 4, Colors::Aqua, -1);
                cv::circle(data->dst, data->symbol, 4, Colors::Yellow, -1);
                flag1 = true;
                flag2 = true;

            } else if (object.color == 0 && object.cls == 2) {
                //已激活的符叶，可以用来扩展一张图中的得到的信息数量
                cls = RuneClass::BlueActivated;
                flag3 = true;

            } else if (object.color == 1 && object.cls == 2) {
                //已激活的符叶，可以用来扩展一张图中的得到的信息数量
                cls = RuneClass::RedActivated;
                flag3 = true;
            }
            for (int i = 0; i < 5; i++) { //画出五个关键点
                cv::circle(data->dst, object.vertices[i], 5, Colors::White, -1);
                cv::circle(
                    data->dst,
                    (object.vertices[0] + object.vertices[1] + object.vertices[2]
                     + object.vertices[4])
                        / 4,
                    5,
                    Colors::White,
                    -1
                );
            }
        }
        if (flag1 && flag2) //有R标数据和符叶数据，则认为识别完成
            data->find = true;
        else {
            data->find = false;
        }
        return true;
    }
}
} // namespace rune