#pragma once

#include "nn.h"
namespace rm_auto_aim{

    class Rune {
    public:
        struct Data {
            cv::Point2f symbol;
            cv::Point2f armor;
            std::vector<cv::Point2d> vertices;
        };
        explicit Rune();
        void detect(sptr<Component::Data> data) override;
        struct RuneTracker {
            int id;
            RuneClass runeclass;
            cv::Point2f symbol;
            std::vector<cv::Point2f> vertices;
            cv::Point2f center;
            Timestamp timestamp;
        };

    private:
        bool Neural_Network;
        NeuralNetwork yolo;

        sptr<Data> data;

        cv::Mat src, dst, roi;
        cv::Rect last;
        cv::Mat mask_hsv, mask_gray, mask;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<std::vector<cv::Point>> approximations;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<int>> children;
        std::vector<std::vector<cv::Point>> convex_hulls;
        std::vector<double> areas;
        std::vector<cv::RotatedRect> rects;
        std::vector<std::size_t> possible_leaves;
        std::vector<std::size_t> possible_symbols;

        std::vector<std::size_t> possible_outer_lightbar;
        std::vector<std::size_t> possible_inside_lightbar;

        std::vector<std::size_t> ring;

        std::pair<int, int> matched_lightbar;
        std::vector<cv::RotatedRect> ellipse;

        bool layer_bool;
        int layer1;
        int layer2[5];
        int layer3;
        int layer4;

        int outer_lightbar;
        int inside_lightbar;
        int symbol;
        // gray滤亮度,color提高颜色阈值
        uchar BLUE_GRAY_THRESH;
        uchar BLUE_COLOR_THRESH;
        uchar RED_GRAY_THRESH;
        uchar RED_COLOR_THRESH;

        uchar COUNT = 0;

        int contour_size;

        bool Prepare();
        bool Binarize();
        bool FindContours();
        bool Filter();
        bool Select();
        bool Closeout();
        bool Yolo();

        std::vector<NeuralNetwork::RuneObject> objects;
        double CONFIDENCE;
        bool is_last_target_exists;
        int lost_cnt;
        double last_target_area;
    };

}  // namespace phoenix::detector
