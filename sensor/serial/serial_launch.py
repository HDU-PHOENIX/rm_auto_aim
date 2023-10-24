from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="serial",
            executable="serial",
            name="serial",
            output="screen",
            parameters=[
                {"baud_rate":115200}
                {"device_name" : "/dev/ttyUSB0"},
                {"flow_control":FlowControl::NONE}
                {"parity":Parity::NONE}
                {"stop_bits":StopBits::ONE}
            ]
        )
    ])
