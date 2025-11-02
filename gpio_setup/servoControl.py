import board
from adafruit_pca9685 import PCA9685

print(f"i2c pins:\n\tSDA={board.SDA}\n\tSCL={board.SCL}")
i2c = board.I2C()  # uses board.SCL and board.SDA
pca = PCA9685(i2c)
pca.frequency = 1e2

pca.channels[0].duty_cycle = 0x7FFF
