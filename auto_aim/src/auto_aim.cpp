#include "armor_detector/detector_node.hpp"
#include "camera/camera_node.hpp"

int main(int argc, char* argv[]) {
    // 初始化
    rclcpp::init(argc, argv);

    // 创建单线程执行器，用于 camera_node 和 detector_node 进行进程间通信
    rclcpp::executors::SingleThreadedExecutor executor;
    // 为 camera_node 和 detector_node 创建节点选项
    rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true);

    // 创建 camera_node
    std::shared_ptr<CameraNode> camera_node = nullptr;
    try {
        camera_node = std::make_shared<CameraNode>(options);
    } catch (const std::exception& e) {
        fprintf(stderr, "%s Exiting ..\n", e.what());
        return 1;
    }

    // 创建 detector_node
    auto detector_node = std::make_shared<rm_auto_aim::ArmorDetectorNode>(options);

    // 将 camera_node 和 detector_node 添加到单线程执行器中
    executor.add_node(camera_node);
    executor.add_node(detector_node);

    // 执行单线程执行器
    executor.spin();

    // 退出
    rclcpp::shutdown();
    return 0;
}
