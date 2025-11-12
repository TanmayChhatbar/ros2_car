import pyglet

class RacecarSimwheel:
    def __init__(self):
        # Create a hidden window for pyglet to manage events (non-blocking)
        self.window = pyglet.window.Window(visible=False)

        # Get the joystick (wheel)
        joysticks = pyglet.input.get_joysticks()
        assert joysticks, 'No joystick device is connected'
        self.wheel = [j for j in joysticks if 'SIMAGIC Alpha' in j.device.name][0]
        self.wheel.open()

        # Get the pedals (find your pedals by name or other criteria)
        devices = pyglet.input.get_devices()
        self.pedals = [d for d in devices if 'P1000' in d.name][0]
        self.pedals.open()

        # Initialize throttle, brake, steering
        self.throttle = 0
        self.brake = 0
        self.steering = 0
        self.handbrake = False
        self.last_throttle = 0
        self.last_brake = 0
        self.last_steering = 0
        self.last_handbrake = False

    def isNewInput(self):
        # Check if there is any new input since the last update
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
        self.window
        # Read the joystick values (wheel and pedals)
        steering = self.wheel.x
        pedal_controls = self.pedals.get_controls()

        # Map the throttle and brake values (assuming pedal index mapping)
        throttle = max(0, (pedal_controls[2].value) / 4096)
        brake = max(0, (pedal_controls[1].value) / 4096)

        # Deadzone handling for throttle and brake
        def deadzone(value, start, end):
            if abs(value) < start:
                return 0
            elif abs(value) < end:
                return (abs(value) - start) / (end - start) * (1 if value > 0 else -1)
            else:
                return value

        throttle = deadzone(throttle, 0.02, 0.05)
        brake = deadzone(brake, 0.02, 0.05)

        # Update internal state
        self.steering = steering
        self.throttle = throttle
        self.brake = brake

        # Print the values (debugging)
        print(f"Steering: {self.steering:.3f}, Throttle: {self.throttle:.3f}, Brake: {self.brake:.3f}")

