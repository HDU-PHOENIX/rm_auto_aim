# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

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
                parameters = [config],
                remappings=[('/detector/armors', '/detector/armors/left'),
                            ('/shooter_info/left','/shooter_info/left'),
                            ('/detector/ignore_classes', '/detector/ignore_classes/left'),
                            ('/communicate/autoaim','/communicate/autoaim'),
                            ('/image_pub','/image_pub/left'),
                            ('/detector/debug_lights','/detector/debug_lights/left'),
                            ('/detector/debug_armors','/detector/debug_armors/left'),
                            ('/detector/marker','/detector/marker/left'),
                            ('/detector/binary_img','/detector/binary_img/left'),
                            ('/detector/number_img','/detector/number_img/left'),
                            ('/detector/result_img','/detector/result_img/left')]
            ),
            ComposableNode(
                package = 'rune_detector',
                plugin = 'rune::RuneDetectorNode',
                name = 'rune_detector_node',
                extra_arguments = [{"use_intra_process_comms": True}],
                parameters = [config],
                remappings=[('/detector/runes','/detector/runes/left'),
                            ('/rune_detector/marker','/rune_detector/marker/left'),
                            ('/rune_detector/debug_img','/rune_detector/debug_img/left'),
                            ('/shoot_info/left','/shoot_info/left'),
                            ('/communicate/autoaim','/communicate/autoaim'),
                            ('/image_pub','/image_pub/left')]
            ),
            ComposableNode(
                package = 'camera',
                plugin = 'sensor::CameraNode',
                name = 'camera_node',
                extra_arguments = [{"use_intra_process_comms": True}],
                parameters = [config],
                remappings=[('/image_pub','/image_pub/left')]
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
                parameters = [config],
                remappings=[('/tracker/reset','/tracker/reset/left'),
                            ('/detector/armors','/detector/armors/left'),
                            ('/tracker/info','/tracker/info/left'),
                            ('/tracker/target','/tracker/target/left'),
                            ('/tracker/marker','/tracker/marker/left')]
            ),
            ComposableNode(
                package='rune_tracker',
                plugin='rune::RuneTrackerNode',
                name='rune_tracker_node',
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters = [config],
                remappings=[('/detector/runes','/detector/runes/left'),
                            ('/rune_detector/marker','/rune_detector/marker/left'),
                            ('/rune_detector/debug_img','/rune_detector/debug_img/left'),
                            ('/shoot_info/left','/shoot_info/left'),
                            ('/communicate/autoaim','/communicate/autoaim'),
                            ('/image_pub','/image_pub/left')]
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
                parameters = [config],
                remappings=[('/shooter/marker','/shooter/marker/left'),
                            ('/shoot_info/left','/shoot_info/left'),
                            ('/tracker/target','/tracker/target/left')]
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
                parameters = [config],
                remappings=[('/communicate/gyro/left','/communicate/gyro/left')]
            )
        ]
    )

    return launch.LaunchDescription([tf_tree, detector, tracker, shooter])
