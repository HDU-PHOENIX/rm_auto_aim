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
            name='rm_auto_aim',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='serial',
                    plugin='sensor::SerialNode',
                    name="serial_node",
                    parameters=[
                        {'baud_rate':115200},
                        {'device_name' : "/dev/ttyACM0"},
                        {'default_data_recv_start':115},#此处要用ascii码 用字符会报错
                        {'default_data_recv_color':114},
                        {'default_data_recv_mode':114},
                        {'default_data_recv_speed':20.0},
                        {'default_data_recv_euler':[0.0,0.0,0.0]},
                        {'default_data_recv_shootbool':0},
                        {'default_data_recv_runeflag':0},
                        {'default_data_recv_end':101},
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='armor_detector',
                    plugin='armor::ArmorDetectorNode',
                    name='armor_detector_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='rune_detector',
                    plugin='rune::RuneDetectorNode',
                    name='rune_detector_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='camerainfo',
                    plugin='camerainfo::CameraInfoNode',
                    name='camera_info_node',
                    extra_arguments=[{"use_intra_process_comms": True}],
                    parameters=[#相机内参
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
                    ]#相机畸变矩阵
                ),
                ComposableNode(
                    package='camera',
                    plugin='sensor::CameraNode',
                    name='camera_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
