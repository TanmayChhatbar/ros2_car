from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    imu_telemetry_node = Node(
        package='telemetry',
        executable='icm20948_ros'
    )

    ld.add_action(imu_telemetry_node)
    return ld
