from as5600 import AS5600
import time

sensor = AS5600()

def main():
    while True:
        sensor.update()
        angle = sensor.get_angle_degrees()
        velocity = sensor.get_velocity()
        print(f"Angle: {angle:.2f} degrees, Velocity: {velocity:.2f} deg/s")
        time.sleep(0.1)

if __name__ == "__main__":
    main()
