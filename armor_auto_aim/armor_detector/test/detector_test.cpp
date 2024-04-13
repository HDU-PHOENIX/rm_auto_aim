#include "armor_detector/detector.hpp"
#include "camera/mindvision.hpp"
#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

TEST(DetectorTest, TestDetectArmor) {
    // read video
    std::shared_ptr<cv::Mat> frame;
    auto&& pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    std::cout << pkg_path << std::endl;

    sensor::MindVision cap(ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config");

    auto total_time = 0;

    armor::Detector detector(
        armor::DetectorParam {
            { 100, 100 },
            { 100, 100 } },
        pkg_path + "/model/mlp.onnx",
        pkg_path + "/model/label.txt",
        0.7,
        std::vector<double> { 1302.9388992859376, 0, 609.2298064340857, 0, 2515.6912302455735, 467.0345949712323, 0, 0, 1 },
        std::vector<double> { 0.9716178021093913, -22.20834732244382, -0.19838225091062828, -0.08828110807170159, 96.16902256363146 },
        armor::Color::BLUE,
        std::vector<std::string> { "negative" }
    );
    while (cap.GetFrame(frame)) {
        auto start_time = std::chrono::steady_clock::now();
        std::vector<armor::Armor> armors = detector.DetectArmor(*frame);
        auto end_time = std::chrono::steady_clock::now();

        auto detect_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        // std::cout << "Time: " << detect_time << "us" << std::endl;
        total_time += detect_time;

        cv::Mat result(*frame);
        detector.DrawResult(result);
        cv::imshow("result", result);
        cv::waitKey(1);
    }
    // std::cout << "Average time: " << total_time / cap.get(cv::CAP_PROP_FRAME_COUNT) << "us" << std::endl;

    // armor::Detector detector(100, 100, armor::Color::RED);
    // cv::Mat input = cv::imread("../../armor_detector/test/test2.jpg");
    // ASSERT_FALSE(input.empty());
    // std::vector<armor::Armor> armors = detector.DetectArmor(input);
    // // ASSERT_FALSE(armors.empty());
    // cv::Mat result(input);
    // detector.DrawResult(result);
    // cv::imshow("result", result);
    // cv::waitKey(0);
}
