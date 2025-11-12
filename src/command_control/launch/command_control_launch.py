from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    imu_telemetry_node = Node(
        package='command_control',
        executable='joystick_control'
    )

    rqt_image_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        parameters=[{
                'ros_args': ['--remap', '/camera/image_raw/_image_transport:=raw']
            }]
    )

    ld.add_action(imu_telemetry_node)
    ld.add_action(rqt_image_node)
    return ld
