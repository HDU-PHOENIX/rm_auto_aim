#include "CameraApi.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/types_c.h>
#include <rclcpp/node.hpp>
#include <stdio.h>
#include <stdlib.h>

namespace sensor {

class MindVision {
public:
    MindVision();
    ~MindVision();
    bool GetFrame(cv::Mat& frame);

    int i_camera_counts;
    int i_status;
    tSdkCameraDevInfo t_camera_enum_list;
    int h_camera;
    tSdkCameraCapbility t_capability; //设备描述信息
    tSdkFrameHead s_frame_info;
    BYTE* pby_buffer;
};

} // namespace sensor
