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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='armor_detector',
                #     plugin='armor::ArmorDetectorNode',
                #     name='armor_detector_node',
                #     extra_arguments=[{"use_intra_process_comms": True}]
                # ),
                ComposableNode(
                    package="serial",
                    plugin='sensor::SerialNode',
                    name="serial_node",
                    parameters=[
                        {"baud_rate":115200},
                        {"device_name" : "/dev/ttyACM0"},
                        # {"flow_control":FlowControl::NONE},
                        # {"parity":Parity::NONE},
                        # {"stop_bits":StopBits::ONE}
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='rune_detector',
                    plugin='rune::RuneDetectorNode',
                    name='rune_detector_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='camera',
                    plugin='sensor::CameraNode',
                    name='camera_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='camerainfo',
                    plugin='camerainfo::CameraInfoNode',
                    name='camera_info_node',
                    # extra_arguments=[{"use_intra_process_comms": True}]
                    parameters=[
                        {'camera_matrix': [1436.5522013177274,
                                           0,
                                            631.7426656086038,
                                            0,
                                            1436.7519670955378,
                                            479.37777230242415,
                                            0,
                                            0,
                                            1]},
                        {'distortion': [-0.12097647520170836,
                                        0.14048371243276836,
                                        0.0001667597668430485,
                                        -0.0028646864621099328,
                                        -0.05038697039343976]},
                    ],  
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
