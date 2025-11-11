#!/usr/bin/env python3
from icm20948 import ICM20948
from smbus2 import SMBus


def persistentInit():
    n_attempts = 0
    while True:
        try:
            imu = ICM20948(i2c_bus=SMBus(4))
            print("IMU initialized after {:d} attempts".format(n_attempts))
            n_attempts = 0
            return imu
        except Exception as e:
            # print(f'IMU init error: {e}')
            n_attempts += 1
            if n_attempts > 1000:
                break
            pass

def read_imu_sensor(imu):
    try:
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
        orientation = (0.0, 0.0, 0.0, 1.0)  #TODO
        angular_velocity = (gx, gy, gz)
        linear_acceleration = (ax, ay, az)
        print([round(v, 2) for v in [ax, ay, az, gx, gy, gz]])
        return 0

    except IOError as e:
        # print(f'IMU read error: {e}')
        imu = persistentInit()
        return 1
        
    except Exception as e:
        # print(f'Unexpected error: {e}')
        return 1

def main(args=None):
    imu = persistentInit()
    orientation = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
    angular_velocity = (0.0, 0.0, 0.0)  # (x, y, z)
    linear_acceleration = (0.0, 0.0, 9.81)  # (x, y, z)
    angular_velocity_offsets = (2.064448, 0.098061, -0.202982)  # (x, y, z)
    while True:
        read_imu_sensor(imu)

if __name__ == '__main__':
    main()
