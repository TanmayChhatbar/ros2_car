#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from icm20948 import ICM20948
from smbus2 import SMBus


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz
        self.get_logger().info("IMU publisher started.")
        self.n_attempts = 0
        self.imu = self.persistentInit()
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
        self.angular_velocity = (0.0, 0.0, 0.0)  # (x, y, z)
        self.linear_acceleration = (0.0, 0.0, 9.81)  # (x, y, z)
        self.angular_velocity_offsets = (2.064448, 0.098061, -0.202982)  # (x, y, z)

    def persistentInit(self):
        while True:
            try:
                # to use i2c bus 4 on pi 5, add the line:
                # dtoverlay=i2c-gpio,bus=4,i2c_gpio_delay_us=1,i2c_gpio_sda=27,i2c_gpio_scl=22
                # to
                # sudo nano /boot/firmware/config.txt

                imu = ICM20948(i2c_bus=SMBus(4))
                # print("IMU initialized after {:d} attempts".format(self.n_attempts))
                self.n_attempts = 0
                return imu
            except IOError as e:
                self.n_attempts += 1
                if self.n_attempts > 100:
                    break
                pass
    
    def read_imu_sensor(self):
        try:
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
            self.orientation = (0.0, 0.0, 0.0, 1.0)  #TODO
            self.angular_velocity = (gx, gy, gz)
            self.linear_acceleration = (ax, ay, az)
            return 0

        except IOError as e:
            # self.get_logger().error(f'IMU read error: {e}')
            self.imu = self.persistentInit()
            return 1
            
        except Exception as e:
            # self.get_logger().error(f'Unexpected error: {e}')
            return 1
            
    def publish_imu(self):
        if self.read_imu_sensor():
            return
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation = Quaternion(
            x=self.orientation[0],
            y=self.orientation[1],
            z=self.orientation[2],
            w=self.orientation[3]
        )

        msg.angular_velocity = Vector3(
            x=self.angular_velocity[0]-self.angular_velocity_offsets[0],
            y=self.angular_velocity[1]-self.angular_velocity_offsets[1],
            z=self.angular_velocity[2]-self.angular_velocity_offsets[2]
        )

        msg.linear_acceleration = Vector3(
            x=self.linear_acceleration[0],
            y=self.linear_acceleration[1],
            z=self.linear_acceleration[2]
        )

        # Covariance can be filled if known, otherwise -1 means "unknown"
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
