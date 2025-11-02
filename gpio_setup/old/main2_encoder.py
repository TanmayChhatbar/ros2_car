from gpiozero import RotaryEncoder, Servo

import lgpio
encoder = RotaryEncoder(16, 20)
motor = Servo(21, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

def main():
    last_step = 0
    while True:
        new_step = encoder.steps
        if new_step != last_step:
            last_step = new_step
            motor_val = new_step / 16.0
            motor_val = max(-1.0, min(1.0, motor_val))
            motor.value = motor_val
            print(motor_val,"\t", motor.value)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        encoder.close()
        # motor.detach()
