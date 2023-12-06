#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "rune_detector/detector_node.hpp"

#include "rune_detector/colors.hpp"
#include "rune_detector/rune_type.hpp"

#define SEND_DEFAULT_DATA true

using std::placeholders::_1;
using std::placeholders::_2;
namespace rune {
RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("rune_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
    confidence_threshold_ = this->declare_parameter("confidence_threshold", 0.9); // 置信度阈值
    model_path = this->declare_parameter("model_path", "/model/yolox_fp16.onnx"); // 模型路径
    detector_ = InitDetector();                                                   // 初始化神符识别器

    //创建标记发布者
    debug_ = this->declare_parameter("debug", false);

    // 监视 Debug 参数变化
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
        debug_ = p.as_bool();
    });

    // 创建神符信息发布者
    runes_pub_ = this->create_publisher<auto_aim_interfaces::msg::Rune>("/detector/runes", rclcpp::SensorDataQoS());

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    rune_marker_.ns = "armors";
    rune_marker_.action = visualization_msgs::msg::Marker::ADD;
    rune_marker_.type = visualization_msgs::msg::Marker::CUBE;
    rune_marker_.scale.x = 0.05;
    rune_marker_.scale.y = 0.23; //设置默认的x,y,z
    rune_marker_.scale.z = 0.125;
    rune_marker_.color.a = 1.0;
    rune_marker_.color.g = 0.5;
    rune_marker_.color.b = 1.0;
    rune_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/marker", 10);

    // 创建相机信息订阅者
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        cam_info_sub_.reset();
    });

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_for_rune", rclcpp::SensorDataQoS(), std::bind(&RuneDetectorNode::ImageCallback, this, std::placeholders::_1));
}

bool RuneDetectorNode::DetectRunes(const sensor_msgs::msg::Image::SharedPtr& img_msg) {
    auto&& img = cv::Mat(img_msg->height, img_msg->width, CV_8UC3,
                         img_msg->data.data()); // 把图像信息转换为cv::Mat格式

    detector_->detect(img, objects_); // 把神符识别结果放到objects_里面

    runes_msg_.header = rune_marker_.header = img_msg->header;

    rune_marker_.id = 0;
    // RuneClass cls; // 符叶枚举类对象 用于标记符叶的种类
    bool flag1 = false, flag2 = false;     //bool flag3 = false;
    cv::Point2f symbol;                    // 符叶R标的位置
    cv::Point2f rune_armor;                // 符叶未激活装甲板中心
    std::vector<cv::Point2d> rune_points_; // 未激活符叶的五个点
    ///------------------------生成扇叶对象----------------------------------------------
    for (auto object: objects_) {
        // 遍历所有的神符识别结果，把R标和未激活的符叶的信息画出来
        auto prob = object.prob;
        if (prob < confidence_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                        "prob is too low"); // 如果置信度小于阈值，则不进行处理
            continue;
        }

        auto&& detect_center = (object.vertices[0] + object.vertices[1] + object.vertices[2] + object.vertices[3] + object.vertices[4]) / 5; // 用于计算R标位置

        auto&& get_symbol = [](const cv::Point2f& lightbar_mid_point, const cv::Point2f& armor_center, const double& center_lightbar_ratio, const bool& flag) {
            // get_symbol是通过符叶的坐标来计算中心R标的位置
            if (flag == 0) {
                // flag = 0使用装甲板中心和内灯条算出标识符位置
                return ((lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);

            } else if (flag == 1) {
                // flag = 1使用装甲板中心和外灯条算出标识符位置
                return (-(lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);
            }
            return cv::Point2f(0, 0);
        };

        if (object.color == 0 && object.cls == 0) {
            // cls = RuneClass::Blue;
            flag1 = true;
            symbol = detect_center;

        } else if (object.color == 1 && object.cls == 0) {
            // cls = RuneClass::Red;
            flag1 = true;
            symbol = detect_center;

        } else if (object.color == 0 && object.cls == 1) {
            // cls = RuneClass::BlueUnActivated;
            rune_points_.clear();

            rune_points_.push_back(object.vertices[1]);
            rune_points_.push_back(object.vertices[2]);
            rune_points_.push_back(object.vertices[4]);
            rune_points_.push_back(object.vertices[0]);
            auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
            auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
            auto&& armor = (tmp1 + tmp2) / 2; // 装甲板中心
            rune_armor = armor;
            if (!flag1) // 如果yolo没有检测到R标
            {
                // symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2,
                // armor, 3.5542, 0)) / 2;
                symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 4.0542, 0)) / 2;
            }
            flag1 = true;
            flag2 = true;
        } else if (object.color == 1 && object.cls == 1) {
            // cls = RuneClass::RedUnActivated;
            rune_points_.clear();

            rune_points_.push_back(object.vertices[1]);
            rune_points_.push_back(object.vertices[2]);
            rune_points_.push_back(object.vertices[4]);
            rune_points_.push_back(object.vertices[0]);
            auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
            auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
            auto&& armor = (tmp1 + tmp2) / 2;
            rune_armor = armor;
            if (!flag1) {
                // symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2,
                // armor, 3.5542, 0)) / 2;
                symbol = (get_symbol(tmp1, armor, 5.295454, true) + get_symbol(tmp2, armor, 4.0542, false)) / 2;
            } // 如果yolo没有检测到R标
            flag1 = true;
            flag2 = true;

        } else if (object.color == 0 && object.cls == 2) {
            // 已激活的符叶，可以用来扩展一张图中的得到的信息数量
            // cls = RuneClass::BlueActivated;
            //flag3 = true;
            continue;

        } else if (object.color == 1 && object.cls == 2) {
            // 已激活的符叶，可以用来扩展一张图中的得到的信息数量
            // cls = RuneClass::RedActivated;
            //flag3 = true;
            continue;
        }

        for (int i = 0; i < 5; i++) { // 画出五个关键点
            cv::circle(img, object.vertices[i], 5, Colors::Green, -1);
        }
    }

    if (debug_) {
        cv::circle(img, rune_armor, 6, Colors::Green, -1);                            //画出装甲板中心
        cv::circle(img, symbol, 6, Colors::Green, -1);                                //画出R标中心
        cv::circle(img, cv::Point2f(img.rows / 2, img.cols / 2), 2, Colors::Blue, 3); // 图像中心点
        cv::imshow("tmp", img);
        cv::waitKey(1);
    } else {
        cv::destroyAllWindows();
    }
    if (flag1 && flag2) // 有R标数据和符叶数据，则认为识别完成
    {
        RCLCPP_WARN(this->get_logger(), "find R and Rune_armor");
        cv::Mat rvec, tvec;
        bool success = pnp_solver_->SolvePnP(rune_points_, rvec, tvec); // 输出旋转向量和平移向量
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "PnP failed!");
            return false;
        } else {
            // RCLCPP_WARN(this->get_logger(), "PnP success!"); // 识别成功
            //判断大小符 //0为不可激活，1为小符，2为大符
            if (img_msg->header.frame_id == "0") {
                runes_msg_.motion = 0;
            } else if (img_msg->header.frame_id == "1") {
                runes_msg_.motion = 1;
            } else if (img_msg->header.frame_id == "2") {
                runes_msg_.motion = 2;
            }
            runes_msg_.pose_c.position.x = tvec.at<double>(0);
            runes_msg_.pose_c.position.y = tvec.at<double>(1);
            runes_msg_.pose_c.position.z = tvec.at<double>(2); // 未激活符叶 相机坐标系下的位置
            runes_msg_.leaf_dir.x = (rune_armor - symbol).x;
            runes_msg_.leaf_dir.y = (rune_armor - symbol).y; // 符叶向量

            for (int i = 0; i < 4; i++) {
                runes_msg_.rune_points[i].x = rune_points_[i].x;
                runes_msg_.rune_points[i].y = rune_points_[i].y;
            }
            runes_msg_.symbol.x = symbol.x; // R标位置 图像左上角为原点
            runes_msg_.symbol.y = symbol.y; // R标位置 图像左上角为原点
            runes_msg_.header.frame_id = "camera";
            runes_msg_.find = true; // 找到符叶

            // Fill the markers
            rune_marker_.header.frame_id = "camera";
            rune_marker_.id++;
            rune_marker_.pose = runes_msg_.pose_c;
            return true;
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "cant find R and Rune_armor");
        return false;
    }
}

void RuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    // RCLCPP_INFO(this->get_logger(), "Detect Runes CallBack !");
#if SEND_DEFAULT_DATA
    //发送测试数据 默认参数
    DetectRunes(img_msg);
    runes_msg_.motion = 2;
    runes_msg_.leaf_dir.x = 1;
    runes_msg_.leaf_dir.y = 1; // 符叶向量
    runes_msg_.symbol.x = 0;   // R标位置 图像左上角为原点
    runes_msg_.symbol.y = 0;   // R标位置 图像左上角为原点
    runes_msg_.header.frame_id = "camera";
    runes_msg_.rune_points[0].x = 100;
    runes_msg_.rune_points[0].y = 100;
    runes_msg_.rune_points[1].x = 100;
    runes_msg_.rune_points[1].y = 200;
    runes_msg_.rune_points[2].x = 200;
    runes_msg_.rune_points[2].y = 200;
    runes_msg_.rune_points[3].x = 200;
    runes_msg_.rune_points[3].y = 100;
    cv::Mat rvec, tvec;
    std::vector<cv::Point2d> rune_points_;
    rune_points_.emplace_back(100, 100);
    rune_points_.emplace_back(100, 200);
    rune_points_.emplace_back(200, 200);
    rune_points_.emplace_back(200, 100);
    pnp_solver_->SolvePnP(rune_points_, rvec, tvec);
    runes_msg_.pose_c.position.x = tvec.at<double>(0);
    runes_msg_.pose_c.position.y = tvec.at<double>(1);
    runes_msg_.pose_c.position.z = tvec.at<double>(2);
    // Fill the markers
    rune_marker_.header.frame_id = "camera";
    rune_marker_.pose = runes_msg_.pose_c;
    PublishMarkers();                // 发布标记
    runes_pub_->publish(runes_msg_); // 发布神符信息
#else
    if (pnp_solver_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "pnp_solver_ is nullptr");
    } else {
        //检测图片 如果检测到了符叶则发布符叶信息
        if (DetectRunes(img_msg)) {
            PublishMarkers();                // 发布标记
            runes_pub_->publish(runes_msg_); // 发布神符信息
        } else {
            RCLCPP_WARN(this->get_logger(), "DetectRunes find nothing");
        }
    }
#endif
}

void RuneDetectorNode::PublishMarkers() {
    using Marker = visualization_msgs::msg::Marker;
    rune_marker_.action = Marker::ADD;
    marker_pub_->publish(rune_marker_);
}

std::shared_ptr<NeuralNetwork> RuneDetectorNode::InitDetector() {
    auto&& detector = std::make_shared<NeuralNetwork>();
    auto pkg_path = ament_index_cpp::get_package_share_directory("rune_detector");
    auto model_load_path = pkg_path + model_path;
    detector->Init(model_load_path);

    return detector;
}

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneDetectorNode)