import pygame
from racecar_joystick import RacecarJoystick

joystick = RacecarJoystick()

while True:
    joystick.update()
    if joystick.isNewInput():
        print(f"{' ' if joystick.steering >= 0 else ''}{joystick.steering:.2f}, {joystick.throttle:.2f}, {joystick.brake:.2f}")
