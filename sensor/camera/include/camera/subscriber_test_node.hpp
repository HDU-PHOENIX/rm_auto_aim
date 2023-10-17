#ifndef SUBSCRIBER_TEST_NODE_HPP
#define SUBSCRIBER_TEST_NODE_HPP

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

// #include "camera/common.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SubscriberTestNode: public rclcpp::Node {
public:
    SubscriberTestNode();

private:

    rclcpp::Time last_time;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

#endif // SUBSCRIBER_TEST_NODE_HPP