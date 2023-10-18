#ifndef MINDVISION_HPP
#define MINDVISION_HPP

#include "CameraApi.h"

#include "opencv2/core/core.hpp"
#include <opencv2/core/types_c.h>
#include <rclcpp/node.hpp>
#include <stdio.h>
#include <stdlib.h>

class MindVision {
public:
    MindVision();
    ~MindVision();

    /**
     * @brief 获取图像
     * @param frame 用于保存图像的 cv::Mat
     * @return bool 是否成功获取图像
     */
    bool GetFrame(cv::Mat& frame);

    /**
     * @brief 获取图像
     * @param frame 用于保存图像的 std::shared_ptr<cv::Mat>
     * @return bool 是否成功获取图像
     */
    bool GetFrame(std::shared_ptr<cv::Mat>& frame);

private:
    // 相机数量
    int i_camera_counts;
    // 相机状态，在 CameraStatus.h 中查看
    int i_status;

    // 相机其他信息
    tSdkCameraDevInfo t_camera_enum_list;
    int h_camera;
    tSdkCameraCapbility t_capability;
    tSdkFrameHead s_frame_info;
    BYTE* pby_buffer;
    unsigned char* g_p_rgb_buffer; //处理后数据缓存区
};

#endif // MINDVISION_HPP
