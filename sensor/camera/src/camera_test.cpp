#include "camera/camera_node.hpp"
#include "camera/subscriber_test_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    // Connect the nodes as a pipeline: camera_node -> sub_node
    std::shared_ptr<CameraNode> camera_node = nullptr;
    try {
        camera_node = std::make_shared<CameraNode>();
    } catch (const std::exception& e) {
        fprintf(stderr, "%s Exiting ..\n", e.what());
        return 1;
    }
    auto sub_node =
        std::make_shared<SubscriberTestNode>();

    executor.add_node(camera_node);
    executor.add_node(sub_node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
