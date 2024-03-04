#include "armor_detector/detector.hpp"
#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

TEST(DetectorTest, TestDetectArmor) {
    // read video
    cv::Mat frame;
    auto&& pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    std::cout << pkg_path << std::endl;
    cv::VideoCapture cap(pkg_path + "/model/test.mp4");
    // cv::namedWindow("result", cv::WINDOW_NORMAL);
    // cv::resizeWindow("result", 600, 600);

    auto total_time = 0;

    ASSERT_TRUE(cap.isOpened());
    while (cap.read(frame)) {
        armor::Detector detector(
            100,
            100,
            armor::Color::BLUE,
            pkg_path + "/model/mlp.onnx",
            pkg_path + "/model/label.txt",
            0.7,
            std::vector<double> { 1302.9388992859376, 0, 609.2298064340857, 0, 2515.6912302455735, 467.0345949712323, 0, 0, 1 },
            std::vector<double> { 0.9716178021093913, -22.20834732244382, -0.19838225091062828, -0.08828110807170159, 96.16902256363146 },
            std::vector<std::string> { "negative" }
        );
        auto start_time = std::chrono::steady_clock::now();
        std::vector<armor::Armor> armors = detector.DetectArmor(frame);
        auto end_time = std::chrono::steady_clock::now();

        auto detect_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "Time: " << detect_time << "us" << std::endl;
        total_time += detect_time;

        cv::Mat result(frame);
        if (!armors.empty()) {
            detector.DrawResult(result);
        }
        cv::imshow("result", result);
        cv::waitKey(1);
    }
    std::cout << "Average time: " << total_time / cap.get(cv::CAP_PROP_FRAME_COUNT) << "us" << std::endl;

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
