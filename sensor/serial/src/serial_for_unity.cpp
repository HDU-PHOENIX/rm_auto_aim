#include "serial/serial_for_unity.hpp"

namespace sensor {
SerialForUnity::SerialForUnity(const rclcpp::NodeOptions& options):
    Node("serial_for_unity_node", options) {
    broadcaster_camera2shooter_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    broadcaster_shooter2odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tfs_camera2shooter_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tfs_shooter2odom_ = std::make_unique<geometry_msgs::msg::TransformStamped>();

    this->serial_info_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/unity2serial",
        rclcpp::SensorDataQoS(),
        std::bind(&SerialForUnity::SerialInfoCallback, this, std::placeholders::_1)
    );
}

void SerialForUnity::SendTransform(
    const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const tf2::Quaternion& q,
    const tf2::Vector3& v,
    const rclcpp::Time& timestamp
) {
    tfs->header.stamp = timestamp;
    tfs->header.frame_id = frame_id;
    tfs->child_frame_id = child_frame_id;
    tfs->transform.translation.x = v.x();
    tfs->transform.translation.y = v.y();
    tfs->transform.translation.z = v.z();
    tfs->transform.rotation.x = q.x();
    tfs->transform.rotation.y = q.y();
    tfs->transform.rotation.z = q.z();
    tfs->transform.rotation.w = q.w();
    broadcaster->sendTransform(*tfs);
}

void SerialForUnity::SerialInfoCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state) {
    float yaw = joint_state->position[0];
    float pitch = joint_state->position[1];
    // x:red y:green z:blue
    // 发布 相机 到 枪口 的坐标系转换
    SendTransform(
        broadcaster_camera2shooter_,
        tfs_camera2shooter_,
        "camera",
        "shooter",
        // 四元数字和欧拉角转换 https://quaternions.online
        []() {
            tf2::Quaternion q;
            q.setRPY(M_PI_2, -M_PI_2, 0);
            return q;
        }(),
        //TODO::平移向量要看xj发的参数
        tf2::Vector3(0, 0.05, 0.14),
        joint_state->header.stamp
    );
    // 发布 枪口 到 odom 的坐标系转换（补偿 yaw pitch 轴的云台转动）
    SendTransform(
        broadcaster_shooter2odom_,
        tfs_shooter2odom_,
        "shooter",
        "odom",
        [&]() {
            tf2::Quaternion q;
            q.setRPY(0, -pitch, -yaw);
            return q;
        }(),
        //TODO::平移向量要看xj发的参数
        tf2::Vector3(-0.2, 0.1, -0.1),
        joint_state->header.stamp
    );
}
} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::SerialForUnity)