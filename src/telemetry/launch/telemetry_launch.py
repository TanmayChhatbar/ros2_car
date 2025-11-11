from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    imu_telemetry_node = Node(
        package='telemetry',
        executable='icm20948_ros'
    )

    camera_telemetry_node = Node(
        package='camera_ros',
        executable='camera_node',
        parameters=[{
                'orientation': 180,
                'width': 160,
                'height': 120,
                'camera': '/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a'
            }]
    )

    ld.add_action(imu_telemetry_node)
    ld.add_action(camera_telemetry_node)
    return ld
