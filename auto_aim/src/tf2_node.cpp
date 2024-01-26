#include "auto_aim/tf2_node.hpp"
#include <memory>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace auto_aim {

TF2Node::TF2Node(const rclcpp::NodeOptions& options):
    Node("tf2_node", options) {
    this->camera2shooter_tvec_ = declare_parameter(
        "camera2shooter_tvec",
        std::vector<double> { 0.0, 0.0, 0.0 }
    );
    this->shooter2odom_tvec_ = declare_parameter(
        "shooter2odom_tvec",
        std::vector<double> { 0.0, 0.0, 0.0 }
    );

    broadcaster_camera2shooter_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    broadcaster_shooter2odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tfs_camera2shooter_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tfs_shooter2odom_ = std::make_unique<geometry_msgs::msg::TransformStamped>();

    euler_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/communicate/gyro/left",
        rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            builtin_interfaces::msg::Time stamp;
            stamp.set__sec(msg->data[0]);
            stamp.set__nanosec(msg->data[1]);

            // 四元数字和欧拉角转换 https://quaternions.online
            // foxglove x:red y:green z:blue

            // 发布 相机 到 枪口 的坐标系转换
            SendTransform(
                broadcaster_camera2shooter_,
                tfs_camera2shooter_,
                stamp,
                "camera",
                "shooter",
                []() {
                    tf2::Quaternion q;
                    q.setRPY(M_PI_2, -M_PI_2, 0);
                    return q;
                }(),
                tf2::Vector3(camera2shooter_tvec_[0], camera2shooter_tvec_[1], camera2shooter_tvec_[2])
            );
            // 发布 枪口 到 odom 的坐标系转换（补偿 yaw pitch 轴的云台转动）
            SendTransform(
                broadcaster_shooter2odom_,
                tfs_shooter2odom_,
                stamp,
                "shooter",
                "odom",
                [msg]() {
                    tf2::Quaternion q;
                    q.setRPY(M_PI_2, -msg->data[3], -msg->data[2]);
                    return q;
                }(),
                tf2::Vector3(shooter2odom_tvec_[0], shooter2odom_tvec_[1], shooter2odom_tvec_[2])
            );
        }
    );
}

void TF2Node::SendTransform(
    const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
    const rclcpp::Time& timestamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const tf2::Quaternion& q,
    const tf2::Vector3& v
) {
    tfs->header.stamp = timestamp;
    tfs->header.frame_id = frame_id;
    tfs->child_frame_id = child_frame_id;
    tfs->transform.rotation.x = q.getX();
    tfs->transform.rotation.y = q.getY();
    tfs->transform.rotation.z = q.getZ();
    tfs->transform.rotation.w = q.getW();
    tfs->transform.translation.x = v.getX();
    tfs->transform.translation.y = v.getY();
    tfs->transform.translation.z = v.getZ();

    broadcaster->sendTransform(*tfs);
}
} // namespace auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim::TF2Node)