#include "rclcpp/node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

namespace auto_aim {

class TF2Node: public rclcpp::Node {
public:
    explicit TF2Node(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 发布坐标系转换
     * 
     * @param frame_id 当前坐标系
     * @param child_frame_id 子坐标系
     * @param q 四元数
     * @param v 平移向量
     */
    void SendTransform(
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
        const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
        const rclcpp::Time& timestamp,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const tf2::Quaternion& q,
        const tf2::Vector3& v
    );

    // 下位机欧拉角订阅
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr euler_sub_;

    // 用于发布 相机坐标系 到 枪口坐标系 的转换的广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_camera2shooter_;
    // 用于发布 枪口坐标系 到 odom 的转换的广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_shooter2odom_;
    // 从相机坐标系到枪口中心的转换
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_camera2shooter_;
    // 从枪口中心到 odom 坐标系的转换（补偿 yaw pitch 轴的转动以保证 odom 系静止）
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_shooter2odom_;

    std::vector<double> camera2shooter_tvec_; // 相机坐标系到枪口坐标系的平移向量
    std::vector<double> shooter2odom_tvec_;   // 枪口坐标系到 odom 坐标系的平移向量
};
} // namespace auto_aim
