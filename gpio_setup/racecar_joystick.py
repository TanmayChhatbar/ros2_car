import pygame

class RacecarJoystick:
    def __init__(self):
        pygame.init()

        # Initialize the joystick module
        pygame.joystick.init()

        # Get the number of available joysticks
        joystick_count = pygame.joystick.get_count()
        print(joystick_count, "joystick(s) found.")

        # Get the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.throttle = 0
        self.brake = 0
        self.steering = 0
        self.last_throttle = 0
        self.last_brake = 0
        self.last_steering = 0

    def isNewInput(self):
        is_new = (self.last_throttle != self.throttle or
                self.last_brake != self.brake or
                self.last_steering != self.steering)
        self.last_throttle = self.throttle
        self.last_brake = self.brake
        self.last_steering = self.steering
        return is_new

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    self.steering = event.value
                elif event.axis == 4:
                    self.throttle = (event.value + 1) / 2
                    self.throttle = max(0, self.throttle)
                elif event.axis == 5:
                    self.brake = (event.value + 1) / 2
                    self.brake = max(0, self.brake)
                # print(f'{" " if self.steering >= 0 else ""}{self.steering:.2f}, {self.throttle:.2f}, {self.brake:.2f}')
