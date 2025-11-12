import pygame

class RacecarJoystick:
    def __init__(self):
        pygame.init()

        # Initialize the joystick module
        pygame.joystick.init()

        # Get the number of available joysticks
        joystick_count = pygame.joystick.get_count()
        joystick = [j for j in range(joystick_count) if pygame.joystick.Joystick(j).get_name() == "Xbox One S Controller"]
        for j in joystick:
            print(f"Found joystick: {pygame.joystick.Joystick(j).get_name()}")
        if len(joystick) > 0:
            self.joystick = pygame.joystick.Joystick(joystick[0])
        else:
            raise RuntimeError("No Xbox One S Controller connected.")

        # Get the first joystick
        self.joystick.init()

        self.throttle = 0
        self.brake = 0
        self.steering = 0
        self.last_throttle = 0
        self.last_brake = 0
        self.last_steering = 0
        self.handbrake = False
        self.last_handbrake = False
    
    def isNewInput(self):
        is_new = (self.last_throttle != self.throttle or
                self.last_brake != self.brake or
                self.last_steering != self.steering or
                self.last_handbrake != self.handbrake)
        self.last_throttle = self.throttle
        self.last_brake = self.brake
        self.last_steering = self.steering
        self.last_handbrake = self.handbrake
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
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:  # A button
                    self.handbrake = True
                else:
                    print('Unhandled button press: ', event.button)
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 0:  # A button
                    self.handbrake = False
                # print(f'{" " if self.steering >= 0 else ""}{self.steering:.2f}, {self.throttle:.2f}, {self.brake:.2f}')
