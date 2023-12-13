#include "rune_tracker/LifecycleNode.hpp"

LifecycleNode::LifecycleNode(
    const std::string& node_name,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
):
    Node(node_name, options) {
    state_change_service = this->create_service<lifecycle_interfaces::srv::ChangeState>(
        node_name + "/change_state",
        std::bind(&LifecycleNode::ChangeState, this, std::placeholders::_1, std::placeholders::_2)
    );
}

LifecycleNode::LifecycleNode(
    const std::string& node_name,
    const std::string& namespace_,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
):
    Node(node_name, namespace_, options) {
    state_change_service = this->create_service<lifecycle_interfaces::srv::ChangeState>(
        node_name + "/change_state",
        std::bind(&LifecycleNode::ChangeState, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void LifecycleNode::ChangeState(
    const std::shared_ptr<lifecycle_interfaces::srv::ChangeState::Request> request,
    std::shared_ptr<lifecycle_interfaces::srv::ChangeState::Response> response
) {
    switch (request->sendstate) {
        case 0:
            response->recvreturn = OnDeactivate();
            break;
        case 1:
            response->recvreturn = OnActivate();
            break;
        default:
            break;
    }
}
