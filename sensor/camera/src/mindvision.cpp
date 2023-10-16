#include "CameraApi.h" //相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/types_c.h>
// #include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdio.h>
#include <stdlib.h>

#include "camera/mindvision.hpp"

unsigned char* g_p_rgb_buffer; //处理后数据缓存区

namespace sensor {
sensor::MindVision::MindVision(): i_camera_counts(1), i_status(-1) {
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
    printf("state = %d\n", i_status);

    printf("count = %d\n", i_camera_counts);
    //没有连接设备
    if (i_camera_counts == 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No device connect.");
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera);

    //初始化失败
    printf("state = %d\n", i_status);
    if (i_status != CAMERA_STATUS_SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera init failed.");
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera, &t_capability);

    //
    g_p_rgb_buffer = (unsigned char*)malloc(
        t_capability.sResolutionRange.iHeightMax * t_capability.sResolutionRange.iWidthMax * 3
    );
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
    CameraPlay(h_camera);

    /*其他的相机参数设置
        例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
            CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
            CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
            本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
        */

    if (t_capability.sIspCapacity.bMonoSensor != 0) {
        // channel=1;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        // channel=3;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
    }
}

bool sensor::MindVision::GetFrame(cv::Mat& frame) {
    if (CameraGetImageBuffer(h_camera, &s_frame_info, &pby_buffer, 1000) != CAMERA_STATUS_SUCCESS) {
        return false;
    }
    CameraImageProcess(h_camera, pby_buffer, g_p_rgb_buffer, &s_frame_info);

    cv::Mat mat_image(
        cvSize(s_frame_info.iWidth, s_frame_info.iHeight),
        s_frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
        g_p_rgb_buffer
    );
    frame = mat_image;

    // cv::waitKey(5);

    //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
    //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
    CameraReleaseImageBuffer(h_camera, pby_buffer);
    return true;
}

sensor::MindVision::~MindVision() {
    CameraUnInit(h_camera);
    //注意，现反初始化后再free
    free(g_p_rgb_buffer);
}

} // namespace sensor
