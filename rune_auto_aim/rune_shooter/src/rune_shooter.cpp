#include "rune_shooter/rune_shooter.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
using std::placeholders::_1;
// using std::placeholders::_2;
// using std::placeholders::_3;

namespace rune 
{
    RuneShooterNode::RuneShooterNode(const rclcpp::NodeOptions& options):
        rclcpp::Node("rune_shooter", options)
    {
        count = 0;
        shoot_permit = false;
        RCLCPP_INFO(this->get_logger(), "Starting ShooterNode!");
        rune_target_sub_ = this->create_subscription<auto_aim_interfaces::msg::RuneTarget>(
            "RuneTracker2Shooter", 1, std::bind(&RuneShooterNode::topic_callback, this, _1));
    }

    void RuneShooterNode::topic_callback(const auto_aim_interfaces::msg::RuneTarget::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "ShooterCallBack start!");
        auto&& speed = msg->speed;
        double px = 0,py = 0;
        lead.SetHandOffSet(px, py);//�����ֶ�ƫ����
        auto && pw = Eigen::Vector3d(msg->pw.position.x, msg->pw.position.y, msg->pw.position.z);
        auto&& YawAndPitch = lead.DynamicCalcCompensate(pw, speed, Bullet::Small);

        float dx = YawAndPitch(0);  /// 180 * M_PI;
        float dy = YawAndPitch(1);  /// 180 * M_PI;
        RCLCPP_INFO(this->get_logger(), "YawAndPitch: %f, %f", dx, dy);

        auto && delay = msg->delay;
        if ((sqrt(YawAndPitch[0] * YawAndPitch[0] + YawAndPitch[1] * YawAndPitch[1]) < 0.05)) {  //����λ�����͵��ƶ��Ƕ�С��һ��ֵ,�򿪻�
            if (count < 5) {
                count++;
            } 
            else {
                count = 0;
                // if (((rclcpp::Time(msg->header.stamp) - shoot_timestamp).seconds() > 300_ms + delay))  //ȷ���ӵ�����û�зɵ��Ϳ���һǹ
                // {
                //     shoot_permit = true;
                //     shoot_timestamp = msg->header.stamp;
                //     std::cout << "shoot_permit" << std::endl;
                // }
                if (((rclcpp::Time(msg->header.stamp) - shoot_timestamp).seconds() > delay))  //ȷ���ӵ�����û�зɵ��Ϳ���һǹ
                {
                    shoot_permit = true;
                    shoot_timestamp = msg->header.stamp;
                    std::cout << "shoot_permit" << std::endl;
                }
            }
        }



        shoot_permit = false;
    }
}// namespace rune