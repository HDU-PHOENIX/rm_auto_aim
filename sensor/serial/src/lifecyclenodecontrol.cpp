#include "serial/serial_node.hpp"

namespace sensor {
void SerialNode::InitClient() {
    this->state = 0;
    this->armor_detector_client = create_client<lifecycle_interfaces::srv::ChangeState>("armor_detector/change_state");
    this->armor_shooter_client = create_client<lifecycle_interfaces::srv::ChangeState>("armor_shooter/change_state");
    this->armor_tracker_client = create_client<lifecycle_interfaces::srv::ChangeState>("armor_tracker/change_state");
    this->rune_detector_client = create_client<lifecycle_interfaces::srv::ChangeState>("rune_detector/change_state");
    this->rune_shooter_client = create_client<lifecycle_interfaces::srv::ChangeState>("rune_shooter/change_state");
    this->rune_tracker_client = create_client<lifecycle_interfaces::srv::ChangeState>("rune_tracker/change_state");
}
bool SerialNode::SearchService() {
    while (!armor_detector_client->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_INFO(this->get_logger(), "armor detector service not available");
        return false;
    }
    while (!armor_shooter_client->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_INFO(this->get_logger(), "armor shooter service not available");
        return false;
    }
    while (!armor_tracker_client->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_INFO(this->get_logger(), "armor tracker service not available");
        return false;
    }
    while (!rune_detector_client->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_INFO(this->get_logger(), "rune detector service not available");
        return false;
    }
    while (!rune_shooter_client->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_INFO(this->get_logger(), "rune shooter service not available");
        return false;
    }
    while (!rune_tracker_client->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_INFO(this->get_logger(), "rune tracker service not available");
        return false;
    }
    return true;
}

void SerialNode::LifecycleNodeControl(const char mode) {
    auto deactivate_request = std::make_shared<lifecycle_interfaces::srv::ChangeState::Request>();
    auto activate_request = std::make_shared<lifecycle_interfaces::srv::ChangeState::Request>();
    deactivate_request->sendstate = 0;
    activate_request->sendstate = 1;

    if (this->state == 0 && mode == 'a') {
        if (SearchService()) {
            armor_detector_client->async_send_request(activate_request);
            armor_shooter_client->async_send_request(activate_request);
            armor_tracker_client->async_send_request(activate_request);
            this->state = 1;
        }
    } else if (this->state == 0 && mode == 'r') {
        if (SearchService()) {
            rune_detector_client->async_send_request(activate_request);
            rune_shooter_client->async_send_request(activate_request);
            rune_tracker_client->async_send_request(activate_request);
            this->state = 2;
        }
    } else if (this->state == 1 && mode == 'r') {
        if (SearchService()) {
            armor_detector_client->async_send_request(deactivate_request);
            armor_shooter_client->async_send_request(deactivate_request);
            armor_tracker_client->async_send_request(deactivate_request);
            rune_detector_client->async_send_request(activate_request);
            rune_shooter_client->async_send_request(activate_request);
            rune_tracker_client->async_send_request(activate_request);
            this->state = 2;
        }
    } else if (this->state == 2 && mode == 'a') {
        if (SearchService()) {
            rune_detector_client->async_send_request(deactivate_request);
            rune_shooter_client->async_send_request(deactivate_request);
            rune_tracker_client->async_send_request(deactivate_request);
            armor_detector_client->async_send_request(activate_request);
            armor_shooter_client->async_send_request(activate_request);
            armor_tracker_client->async_send_request(activate_request);
            this->state = 1;
        }
    }
}
} // namespace sensor
