import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with multiple components."""
    config = os.path.join(
        get_package_share_directory('auto_aim'),
        'config', 'config.yaml'
    )

    detector = ComposableNodeContainer(
        name='detector',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments = ["--ros-args", "--log-level", "debug"],
        composable_node_descriptions=[
            ComposableNode(
                package = 'armor_detector',
                plugin = 'armor::ArmorDetectorNode',
                name = 'armor_detector_node',
                extra_arguments = [{"use_intra_process_comms": True}],
                parameters = [config]
            ),
            ComposableNode(
                package = 'camera',
                plugin = 'sensor::CameraNode',
                name = 'camera_node',
                extra_arguments = [{"use_intra_process_comms": True}],
                parameters = [config]
            )
        ]
    )

    tracker = ComposableNodeContainer(
        name='tracker',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package = 'armor_tracker',
                plugin = 'armor::ArmorTrackerNode',
                name = 'armor_tracker_node',
                extra_arguments = [{"use_intra_process_comms": True}],
                parameters = [config]
            )
        ]
    )

    shooter = ComposableNodeContainer(
        name='shooter',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='shooter',
                plugin='auto_aim::ShooterNode',
                name='shooter_node',
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters = [config]
            )
        ]
    )

    tf_tree = ComposableNodeContainer(
        name='tf_tree',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='auto_aim',
                plugin='auto_aim::TF2Node',
                name='tf_tree_node',
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters = [config]
            )
        ]
    )

    return launch.LaunchDescription([tf_tree, detector, tracker, shooter])
