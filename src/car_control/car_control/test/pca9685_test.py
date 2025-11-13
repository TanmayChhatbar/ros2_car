import time
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = board.I2C()  
pca = PCA9685(i2c)
pca.frequency = 50

servo7 = servo.Servo(pca.channels[7])

center = 90
rnge = 75
while True:
    for i in range(int(center - rnge/2), int(center + rnge/2)):
        servo7.angle = i
        print(i)
        time.sleep(0.0)
    time.sleep(1)
    for i in range(int(center + rnge/2), int(center - rnge/2), -1):
        servo7.angle = i
        print(i)
        time.sleep(0.0)
    time.sleep(1)

pca.deinit()
