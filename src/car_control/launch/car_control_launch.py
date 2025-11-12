from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    car_control_node = Node(
        package='car_control',
        executable='car_control'
    )

    imu_telemetry_node = Node(
        package='telemetry',
        executable='icm20948_ros'
    )

    fps = 24
    fdl = int(1000000 / fps)
    camera_telemetry_node = Node(
        package='camera_ros',
        executable='camera_node',
        parameters=[{
                'orientation': 180,
                'width': 160,
                'height': 120,
                'camera': '/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a',
                'format': 'XRGB8888',
                'FrameDurationLimits': [fdl,fdl]
            }]
    )

    ld.add_action(car_control_node)
    ld.add_action(imu_telemetry_node)
    ld.add_action(camera_telemetry_node)
    return ld
