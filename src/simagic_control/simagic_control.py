import pyglet

joysticks = pyglet.input.get_joysticks()
assert joysticks, 'No joystick device is connected'
wheel = joysticks[0]
wheel.open()

devices = pyglet.input.get_devices()
pedals = [d for d in devices if 'P1000' in d.name][0]
pedals.open()

window = pyglet.window.Window(width=800, height=800, visible=False)
batch = pyglet.graphics.Batch()

@window.event
def on_draw():
    window.clear()
    batch.draw()
    steering = wheel.x
    pedal_controls = pedals.get_controls()
    throttle = max(0, (pedal_controls[2].value) / (4096))
    brake = max(0, (pedal_controls[1].value) / (4096))

    deadzone_throttle_start = 0.02
    deadzone_throttle_end = 0.05
    deadzone_brake_start = 0.02
    deadzone_brake_end = 0.05

    def deadzone(value, deadzone_start, deadzone_end):
        if abs(value) < deadzone_start:
            return 0
        elif abs(value) < deadzone_end:
            return (abs(value) - deadzone_start) / (deadzone_end - deadzone_start) * (1 if value > 0 else -1)
        else:
            return value
    
    throttle = deadzone(throttle, deadzone_throttle_start, deadzone_throttle_end)
    brake = deadzone(brake, deadzone_brake_start, deadzone_brake_end)

    print(f"Steering: {steering:.3f}, Throttle: {throttle:.3f}, Brake: {brake:.3f}")

pyglet.app.run()
