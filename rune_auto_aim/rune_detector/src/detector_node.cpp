#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/types.hpp>
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
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

// #include "armor_detector/armor.hpp"
#include "rune_detector/detector_node.hpp"

#include "rune_detector/rune_type.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace rune {
RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("rune_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
    image_sub.subscribe(this, "/image_raw");
    serial_sub.subscribe(this, "/serial_info");
    sync_ = std::make_shared<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image,
        auto_aim_interfaces::msg::SerialInfo>>(image_sub, serial_sub, 1);

    sync_->registerCallback(std::bind(&RuneDetectorNode::topic_callback, this, _1, _2));
    // 初始化神符识别器
    confidence_threshold_ = 0.7;
    detector_ = InitDetector();

    // 创建标记发布者
    debug_ = this->declare_parameter("debug", false);
    if (debug_) {
        CreateDebugPublishers();
    }

    // 创建神符信息发布者
    runes_pub_ = this->create_publisher<auto_aim_interfaces::msg::Rune>(
        "/detector/runes",
        rclcpp::SensorDataQoS()
    );

    // 创建相机信息订阅者
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
            cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
            cam_info_sub_.reset();
        }
    );

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&RuneDetectorNode::ImageCallback, this, std::placeholders::_1)
    );
}

bool RuneDetectorNode::DetectRunes(const sensor_msgs::msg::Image::SharedPtr& img_msg) {
    auto&& img = cv::Mat(
        img_msg->height,
        img_msg->width,
        CV_8UC3,
        img_msg->data.data()
    ); //把图像信息转换为cv::Mat格式

    detector_->detect(img, objects_); //把神符识别结果放到objects_里面

    RuneClass cls; //符叶枚举类对象 用于标记符叶的种类
    bool flag1 = false, flag2 = false, flag3 = false;
    cv::Point2f symbol; //符叶R标的位置
    cv::Point2f rune_armor; //符叶未激活装甲板中心
    std::vector<cv::Point2d> rune_points_; //未激活符叶的五个点
    ///------------------------生成扇叶对象----------------------------------------------
    for (auto object: objects_) {
        //遍历所有的神符识别结果，把R标和未激活的符叶的信息画出来
        auto prob = object.prob;
        if (prob < confidence_threshold_) {
            RCLCPP_WARN(this->get_logger(), "prob is too low"); //如果置信度小于阈值，则不进行处理
            continue;
        }

        auto&& detect_center = (object.vertices[0] + object.vertices[1] + object.vertices[2]
                                + object.vertices[3] + object.vertices[4])
            / 5; //用于计算R标位置

        auto&& get_symbol = [](const cv::Point2f& lightbar_mid_point,
                               const cv::Point2f& armor_center,
                               const double& center_lightbar_ratio,
                               const bool& flag) {
            //get_symbol是通过符叶的坐标来计算中心R标的位置
            if (flag == 0) {
                //flag = 0使用装甲板中心和内灯条算出标识符位置
                return ((lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);

            } else if (flag == 1) {
                //flag = 1使用装甲板中心和外灯条算出标识符位置
                return (
                    -(lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center
                );
            }
            return cv::Point2f(0, 0);
        };

        if (object.color == 0 && object.cls == 0) {
            cls = RuneClass::Blue;
            flag1 = true;
            symbol = detect_center;

        } else if (object.color == 1 && object.cls == 0) {
            cls = RuneClass::Red;
            flag1 = true;
            symbol = detect_center;

        } else if (object.color == 0 && object.cls == 1) {
            cls = RuneClass::BlueUnActivated;
            rune_points_.clear();

            rune_points_.push_back(object.vertices[1]);
            rune_points_.push_back(object.vertices[2]);
            rune_points_.push_back(object.vertices[4]);
            rune_points_.push_back(object.vertices[0]);
            auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
            auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
            auto&& armor = (tmp1 + tmp2) / 2; //装甲板中心
            rune_armor = armor;
            if (!flag1) //如果yolo没有检测到R标
            {
                // symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 3.5542, 0)) / 2;
                symbol =
                    (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 4.0542, 0)) / 2;
            }
            cv::circle(img, armor, 4, Colors::Aqua, -1);
            cv::circle(img, symbol, 4, Colors::Yellow, -1);
            flag1 = true;
            flag2 = true;
        } else if (object.color == 1 && object.cls == 1) {
            cls = RuneClass::RedUnActivated;
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
                // symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 3.5542, 0)) / 2;
                symbol =
                    (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 4.0542, 0)) / 2;
            } //如果yolo没有检测到R标

            //data->armor = armor;
            cv::circle(img, armor, 6, Colors::Aqua, -1);
            cv::circle(img, symbol, 6, Colors::Yellow, -1);
            flag1 = true;
            flag2 = true;

        } else if (object.color == 0 && object.cls == 2)
        { //已激活的符叶，可以用来扩展一张图中的得到的信息数量
            cls = RuneClass::BlueActivated;
            flag3 = true;

        } else if (object.color == 1 && object.cls == 2)
        { //已激活的符叶，可以用来扩展一张图中的得到的信息数量
            cls = RuneClass::RedActivated;
            flag3 = true;
        }

        for (int i = 0; i < 5; i++) { //画出五个关键点
            cv::circle(img, object.vertices[i], 5, Colors::White, -1);
        }
        cv::circle(
            img,
            (object.vertices[0] + object.vertices[1] + object.vertices[2] + object.vertices[4]) / 4,
            5,
            Colors::White,
            -1
        );
    }
    if (flag1 && flag2) //有R标数据和符叶数据，则认为识别完成
    {
        //data->find = true;
        RCLCPP_WARN(this->get_logger(), "find R and Rune_armor");
    } else {
        //data->find = false;
        RCLCPP_WARN(this->get_logger(), "cant find R and Rune_armor");
        return false;
    }

    cv::Mat rvec, tvec;
    bool success = pnp_solver_->SolvePnP(rune_points_, rvec, tvec); //输出旋转向量和平移向量
    if (!success) {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
        runes_msg_.find = false; //没找到符叶
        runes_msg_.header = img_msg->header; //包含时间戳
        // runes_msg_.motion = ;//判断大小符
        return false;
    } else {
        RCLCPP_WARN(this->get_logger(), "PnP success!");

        runes_msg_.pose_c.position.x = tvec.at<double>(0);
        runes_msg_.pose_c.position.y = tvec.at<double>(1);
        runes_msg_.pose_c.position.z = tvec.at<double>(2); //未激活符叶相机坐标系下的位置
        runes_msg_.leaf_dir.x = (rune_armor - symbol).x;
        runes_msg_.leaf_dir.y = (rune_armor - symbol).y; //符叶向量
        for (int i = 0; i < 4; i++) {
            runes_msg_.rune_points[i].x = rune_points_[i].x;
            runes_msg_.rune_points[i].y = rune_points_[i].y;
        }
        runes_msg_.symbol.x = symbol.x; //R标位置 图像左上角为原点
        runes_msg_.symbol.y = symbol.y; //R标位置 图像左上角为原点
        runes_msg_.header = img_msg->header; //包含时间戳
        runes_msg_.find = true; //找到符叶
        // runes_msg_.motion = ;//判断大小符
        return true;
    }
}

void RuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    RCLCPP_INFO(
        this->get_logger(),
        "timestamp: %d image address in detector %p",
        img_msg->header.stamp.nanosec,
        static_cast<void*>(const_cast<sensor_msgs::msg::Image*>(img_msg.get()))
    );

    DetectRunes(img_msg); //将图片检测

    runes_pub_->publish(runes_msg_);
}

void RuneDetectorNode::topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
    const auto_aim_interfaces::msg::SerialInfo::ConstSharedPtr& serial_msg
) {
    RCLCPP_INFO(this->get_logger(), "receive!");
    RCLCPP_INFO_STREAM(this->get_logger(), "receive!again");
}

std::shared_ptr<NeuralNetwork> RuneDetectorNode::InitDetector() {
    auto&& detector = std::make_shared<NeuralNetwork>();
    auto pkg_path = ament_index_cpp::get_package_share_directory("rune_detector");
    auto model_path = pkg_path + "/model/Rune/model_15/yolox_fp16.onnx";
    detector->Init(model_path);

    return detector;
}

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneDetectorNode)