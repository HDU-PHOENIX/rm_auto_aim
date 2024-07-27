#include "camera/mindvision.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

TEST(camera, camera_frame_get) {
    std::shared_ptr<cv::Mat> frame;
    int frame_count = 0;
    double avg_fps = 0;
    auto last_frame_time = std::chrono::system_clock::now();
    auto camera = std::make_shared<sensor::MindVision>();

    while (frame_count++ < 1000) {
        ASSERT_TRUE(camera->GetFrame(frame));
        auto current_frame_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time = current_frame_time - last_frame_time;
        double fps = 1.0 / elapsed_time.count();
        std::cout << frame_count << " frame "
                  << "fps: " << fps << std::endl;
        last_frame_time = current_frame_time;
        avg_fps += fps;
    }
    avg_fps /= frame_count;
    ASSERT_GE(avg_fps, 180); // 断言帧率不低于 180
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
