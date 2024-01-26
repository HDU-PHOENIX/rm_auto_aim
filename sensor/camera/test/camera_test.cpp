#include "camera/mindvision.hpp"
#include <ctime>
#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

TEST(camera, camera_frame_get) {
    cv::Mat frame;
    int frame_count = 0;
    clock_t last_frame_time = std::clock();
    auto camera = std::make_shared<sensor::MindVision>();

    while (frame_count++ < 1000) {
        camera->GetFrame(frame);
        clock_t current_frame_time = std::clock();
        double elapsed_time = static_cast<double>(current_frame_time - last_frame_time) / CLOCKS_PER_SEC;
        double fps = 1.0 / elapsed_time;
        std::cout << frame_count << " frame "
                  << "fps: " << fps << std::endl;
        last_frame_time = current_frame_time;

        // 添加一些断言
        ASSERT_FALSE(frame.empty()); // 断言帧不为空
    }
}
