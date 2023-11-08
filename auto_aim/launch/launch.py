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
                        {'default_data_recv_start':115},
                        {'default_data_recv_color':114},
                        {'default_data_recv_mode':114}, # r:114 a:97
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
                    extra_arguments=[{"use_intra_process_comms": True}],
                    parameters=[
                        {'debug': True}
                    ]
                ),
                ComposableNode(
                    package='armor_tracker',
                    plugin='armor::ArmorTrackerNode',
                    name='armor_tracker_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='rune_detector',
                    plugin='rune::RuneDetectorNode',
                    name='rune_detector_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='rune_tracker',
                    plugin='rune::RuneTrackerNode',
                    name='rune_tracker_node',
                    extra_arguments=[{"use_intra_process_comms": True}]
                ),
                ComposableNode(
                    package='camerainfo',
                    plugin='camerainfo::CameraInfoNode',
                    name='camera_info_node',
                    extra_arguments=[{"use_intra_process_comms": True}],
                    parameters=[
                        {'camera_matrix': [2045.7299044722633,
                                           0,
                                           639.91373901944,
                                           0,
                                           2041.4000138988533,
                                           505.5792498148916,
                                           0,
                                           0,
                                           1]},
                        {'distortion': [-0.04513949433516694,
                                        -0.9392361285963754,
                                        -0.0017701462462787585,
                                        -0.0016870244604070346,
                                        8.479602455636757]},
                    ]
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
