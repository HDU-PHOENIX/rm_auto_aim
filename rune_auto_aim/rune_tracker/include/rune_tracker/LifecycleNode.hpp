#ifndef LIFECYCLENODE_HPP_
#define LIFECYCLENODE_HPP_

#include "lifecycle_interfaces/srv/change_state.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

// enum class LifecycleState {
//     inactive, //0
//     active //1
// };

// enum class LifecycleReturn {
//     successful, //0
//     failure //1
// };

class LifecycleNode: public rclcpp::Node {
public:
    explicit LifecycleNode(const std::string& node_name, const rclcpp::NodeOptions& options);

    explicit LifecycleNode(
        const std::string& node_name,
        const std::string& namespace_,
        const rclcpp::NodeOptions& options
    );

    virtual int OnActivate() = 0;

    virtual int OnDeactivate() = 0;

private:
    void ChangeState(
        const std::shared_ptr<lifecycle_interfaces::srv::ChangeState::Request> request,
        std::shared_ptr<lifecycle_interfaces::srv::ChangeState::Response> response
    );
    rclcpp::Service<lifecycle_interfaces::srv::ChangeState>::SharedPtr state_change_service;
};

#endif // LIFECYCLENODE_HPP_