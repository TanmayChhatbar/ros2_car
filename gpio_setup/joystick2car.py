import board
from adafruit_pca9685 import PCA9685

import pygame
from racecar_joystick import RacecarJoystick

print(f"i2c pins:\n\tSDA={board.SDA}\n\tSCL={board.SCL}")
i2c = board.I2C()  # uses board.SCL and board.SDA
pca = PCA9685(i2c)
pca.frequency = 1e2

steering = servo.Servo(pca.channels[7])
throttle = servo.Servo(pca.channels[6])

joystick = RacecarJoystick()

def main():
    while True:
        joystick.update()
        if joystick.isNewInput():
            print(f"{' ' if joystick.steering >= 0 else ''}{joystick.steering:.2f}, {joystick.throttle:.2f}, {joystick.brake:.2f}")
            steering.value = joystick.steering
            throttle.value = joystick.throttle - joystick.brake

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('exiting')
        steering.value = 0
        throttle.value = 0
        pca.deinit()
        pygame.quit()
