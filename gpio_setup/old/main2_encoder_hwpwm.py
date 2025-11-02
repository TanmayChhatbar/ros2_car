from gpiozero import RotaryEncoder
from rpi_hardware_pwm import HardwarePWM

encoder = RotaryEncoder(16, 20)
motor = HardwarePWM(pwm_channel=0, hz=50, chip=1)

pwm_frequency = 50

def hwServo(value):
    # value between -1 and 1
    # duty cycle between 0.5 and 2.5 ms
    period = 1000.0 / pwm_frequency  # period in ms
    min_dc = (0.5 / period) * 100.0
    max_dc = (2.5 / period) * 100.0
    dc = ((value + 1) / 2.0) * (max_dc - min_dc) + min_dc
    motor.change_duty_cycle(dc)
    return dc

def main():
    last_step = 0
    while True:
        new_step = encoder.steps
        motor.start(1.5 / 1000 * pwm_frequency * 100) # full duty cycle
        if new_step != last_step:
            last_step = new_step
            motor_val = new_step / 16.0
            motor_val = max(-1.0, min(1.0, motor_val))
            hwServo(motor_val)
            print(motor_val,"\t", motor._duty_cycle)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('exiting')
        encoder.close()
        # motor.detach()
        motor.stop()

