#include "rclcpp/node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include <sensor_msgs/msg/joint_state.hpp>

namespace auto_aim {

class TF2Node: public rclcpp::Node {
public:
    explicit TF2Node(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 发布坐标系转换，先平移再旋转
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr euler_sub_;

    // 广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_odom2shooter_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_shooter2camera_;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_odom2shooter_;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_shooter2camera_;

    std::vector<double> shooter2camera_tvec_; // 枪口坐标系到相机坐标系的平移向量
    double odom2shooter_r_;                   // 枪口到 odom 坐标系的距离

    // 坐标系名称
    std::string shooter_coordinate, camera_coordinate, odom_coordinate;
};
} // namespace auto_aim
