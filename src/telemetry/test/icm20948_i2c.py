#!/usr/bin/env python

import time
from icm20948 import ICM20948



def persistentInit():
    time_start = time.time()
    n_attempts = 0
    while True:
        try:
            imu = ICM20948()
            print("IMU initialized after {:.4f} seconds and {:d} attempts".format(time.time() - time_start, n_attempts))
            return imu
        except:
            n_attempts += 1
            pass
# end persistentInit()

imu = persistentInit()
last_read_time = time.time()
n = 0
t_update = 0.5

while True:
    try:
        # x, y, z = imu.read_magnetometer_data() # very slow
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
        n += 1
        dt = time.time() - last_read_time
        if dt > t_update:
            last_read_time = time.time()
            print("""
        dt:    {:05.4f} ms
        f:     {:.2f} Hz
        Accel: {:05.2f} {:05.2f} {:05.2f}
        Gyro:  {:05.2f} {:05.2f} {:05.2f}""".format(
                dt/n, n/dt, ax, ay, az, gx, gy, gz
                ))
            n = 0
        time.sleep(0.01)
    except:
        print('error: ', n)
        imu = persistentInit()
