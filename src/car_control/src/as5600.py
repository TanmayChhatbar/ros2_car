from smbus2 import SMBus
import time
from math import pi

class AS5600:
    def __init__(self, i2c_addr=0x36, i2c_bus=SMBus(1), sensitivity_deg_per=360/4096, lpf_cutoff=10.0):
        self.i2c_addr = i2c_addr
        self.i2c_bus = i2c_bus
        self.sensitivity_deg_per = sensitivity_deg_per
        self.lpf_cutoff = lpf_cutoff

        self.__raw_angle_reg = 0x0C
        self.__angle_reg = 0x0E
        
        self.__prev_angle = None
        self.__prev_time = None
        self.__raw_velocity = 0.0
        self.__velocity = 0.0

        self.__calibration_bitval = None #TODO
        self.__calibration_deg = None
    

    def __read_raw_angle(self):
        # bytes
        raw_data = self.i2c_bus.read_i2c_block_data(self.i2c_addr, self.__raw_angle_reg, 2)
        raw_angle = (raw_data[0] << 8) | raw_data[1]
        return raw_angle & 0x0FFF  # Mask to 12 bits
    
    def __read_angle(self):
        # bytes
        angle_data = self.i2c_bus.read_i2c_block_data(self.i2c_addr, self.__angle_reg, 2)
        angle = (angle_data[0] << 8) | angle_data[1]
        return angle & 0x0FFF  # Mask to 12 bits
    
    def get_raw_angle_degrees(self):
        # degrees
        raw_angle = self.__read_raw_angle()
        angle_degrees = raw_angle * self.sensitivity_deg_per
        return angle_degrees

    def get_angle_degrees(self):
        # degrees
        raw_angle = self.__read_angle()
        angle_degrees = raw_angle * self.sensitivity_deg_per
        return angle_degrees

    def update(self):
        # run this periodically to update velocity. ideally at a fixed rate
        current_time = time.time()
        current_angle = self.get_angle_degrees()

        if self.prev_angle is None:
            self.prev_angle = current_angle
            self.prev_time = current_time
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0:
            return self.velocity

        prev_angle = self.prev_angle
        if current_angle < 45 and prev_angle > 315:
            prev_angle -= 360
        elif current_angle > 315 and prev_angle < 45:
            prev_angle += 360

        self.raw_velocity = (current_angle - prev_angle) / dt
        
        # Low-pass filter
        alpha = dt / (dt + (1 / (2 * pi* self.lpf_cutoff)))
        self.velocity = alpha * self.raw_velocity + (1 - alpha) * self.velocity

        self.prev_angle = current_angle
        self.prev_time = current_time



    def get_raw_velocity(self):
        # degrees per second
        return self.raw_velocity

    def get_velocity(self):
        # degrees per second
        return self.velocity

