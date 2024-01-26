#include "camera/mindvision.hpp"
#include <ctime>
#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

TEST(camera, camera_frame_get) {
    cv::Mat frame;
    int frame_count = 0;
    clock_t last_frame_time;
    auto camera = std::make_shared<sensor::MindVision>();

    while (frame_count++ < 1000) {
        camera->GetFrame(frame);
        double fps = 1000.0 / (std::clock() - last_frame_time);
        std::cout << "fps: " << fps << std::endl;
        last_frame_time = std::clock();

        // 添加一些断言
        ASSERT_FALSE(frame.empty()); // 断言帧不为空
        ASSERT_GE(fps, 100);         // 断言帧率不低于最小预期帧率
    }
}
