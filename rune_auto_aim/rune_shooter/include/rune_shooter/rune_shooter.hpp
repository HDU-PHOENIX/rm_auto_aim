#pragma once

#include <rclcpp/rclcpp.hpp>
#include "auto_aim_interfaces/msg/rune_target.hpp"
#include "lead.hpp"//引入lead弹道解算类
// #include "coordinate.h"
namespace rune
{
    class RuneShooterNode : public rclcpp::Node
    {
    public:
        explicit RuneShooterNode(const rclcpp::NodeOptions& options);

    private:
        //接收tracker节点发布的神符信息，然后解算弹道
        rclcpp::Subscription<auto_aim_interfaces::msg::RuneTarget>::SharedPtr rune_target_sub_;
        void topic_callback(const auto_aim_interfaces::msg::RuneTarget::SharedPtr msg);
        bool shoot_permit;
        int count;
        Lead lead;//弹道解算类
        rclcpp::Time shoot_timestamp;
        // std::shared_ptr<Coordinate> coordinate;//坐标转换类
    };
};