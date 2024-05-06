#include "camera/mindvision.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include <opencv2/core/types_c.h>
#include <rclcpp/utilities.hpp>

#define MAX_CAMERA_COUNT 4

namespace sensor {

MindVision::MindVision(std::string mindvision_config, const std::string sn):
    i_camera_counts(2),
    i_status(-1) {
    // 相机 SDK 初始化
    CameraSdkInit(1);

    auto logger = rclcpp::get_logger("camera_node");

    // 枚举设备，并建立设备列表
    i_status = CameraEnumerateDevice(t_camera_enum_list, &i_camera_counts);
    int camera_index = 0;
    if (!sn.empty()) {
        for (int i = 0; i < i_camera_counts; i++) {
            if (std::strcmp(t_camera_enum_list[i].acSn, sn.data()) == 0) {
                RCLCPP_INFO_STREAM(logger, t_camera_enum_list[i].acSn);
                camera_index = i;
                RCLCPP_INFO_STREAM(logger, "camera_index: " << camera_index);
            }
        }
    }
    // 没有连接设备
    if (i_camera_counts == 0) {
        RCLCPP_WARN(logger, "No device connect.");
        int reconnect_times = 5;
        while ((reconnect_times--) != 0) {
            i_status = CameraEnumerateDevice(t_camera_enum_list, &i_camera_counts);
            if (i_camera_counts == 0) {
                RCLCPP_WARN(logger, "No device connect.");
            } else {
                break;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        if (i_camera_counts == 0) {
            RCLCPP_ERROR(logger, "Reconnect failed. State = %d", i_status);
            return;
        } else {
            RCLCPP_INFO(logger, "Reconnect success.");
        }
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(&t_camera_enum_list[camera_index], -1, -1, &h_camera);

    // 初始化失败
    if (i_status != CAMERA_STATUS_SUCCESS) {
        RCLCPP_WARN(logger, "Camera init failed.");

        int reinit_times = 5;
        while ((reinit_times--) != 0) {
            i_status = CameraInit(&t_camera_enum_list[camera_index], -1, -1, &h_camera);
            if (i_status == CAMERA_STATUS_SUCCESS) {
                break;
            } else {
                RCLCPP_WARN(logger, "Camera Init failed.");
            }
        }

        if (i_status == CAMERA_STATUS_SUCCESS) {
            RCLCPP_INFO(logger, "Camera reinit success.");
        } else {
            RCLCPP_ERROR(logger, "Camera reinit failed. State = %d", i_status);
            return;
        }
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera, &t_capability);

    // 创建 rgb 图缓存区
    g_p_rgb_buffer = (unsigned char*)malloc(
        t_capability.sResolutionRange.iHeightMax * t_capability.sResolutionRange.iWidthMax * 3
    );

    // 让 SDK 进入工作模式，开始接收来自相机发送的图像数据。如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像。

    CameraPlay(h_camera);

    // TODO：老项目中还有设置预览的分辨率 ,之后如果出问题了再去看看

    /*
        其他的相机参数设置
        例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
            CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
            CameraSetGamma、CameraSetConrast、CameraSetGain 等设置图像伽马、对比度、RGB 数字增益等等。
    */
    if (mindvision_config.empty()) {
        CameraSetAeState(h_camera, 0);
        CameraSetExposureTime(h_camera, 5000);
        CameraSetContrast(h_camera, 100);
    } else {
        CameraReadParameterFromFile(h_camera, mindvision_config.data());
    }

    if (t_capability.sIspCapacity.bMonoSensor != 0) {
        // channel=1;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        // channel=3;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
    }
}

bool MindVision::GetFrame(std::shared_ptr<cv::Mat>& frame) {
    // 获取缓存区图像
    int code = CameraGetImageBuffer(h_camera, &s_frame_info, &pby_buffer, 1000);
    if (code != CAMERA_STATUS_SUCCESS) {
        auto logger = rclcpp::get_logger("camera_node");
        RCLCPP_ERROR(logger, "error code %d", code);
        return false;
    }

    // 图像处理
    CameraImageProcess(h_camera, pby_buffer, g_p_rgb_buffer, &s_frame_info);

    // 构造
    frame = std::make_shared<cv::Mat>(
        cvSize(s_frame_info.iWidth, s_frame_info.iHeight),
        s_frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
        g_p_rgb_buffer
    );

    //在成功调用 CameraGetImageBuffer 后，必须调用 CameraReleaseImageBuffer 来释放获得的 buffer。
    //否则再次调用 CameraGetImageBuffer 时，程序将被挂起一直阻塞，直到其他线程中调用 CameraReleaseImageBuffer 来释放了 buffer
    CameraReleaseImageBuffer(h_camera, pby_buffer);
    return true;
}

MindVision::~MindVision() {
    CameraUnInit(h_camera);
    //注意，现反初始化后再 free
    free(g_p_rgb_buffer);
}

} // namespace sensor
