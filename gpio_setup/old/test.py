from rpi_hardware_pwm import HardwarePWM
import os
import time
chip = 1
pwm_channel = 0
chip_path = f"/sys/class/pwm/pwmchip{chip}/"
pwm_path = f"{chip_path}/pwm{pwm_channel}"

# Disable first
# if os.path.exists(f"{pwm_path}/export"):
#     with open(f"{chip_path}/export", "w") as f:
#         f.write(f"0\n")

# if os.path.exists(f"{pwm_path}/enable"):
#     with open(f"{pwm_path}/enable", "r") as f:
#         enabled = f.read().strip()
#         if enabled == "1":
#             f.write("0\n")


pwm = HardwarePWM(pwm_channel=pwm_channel, hz=50, chip=chip)
pwm.start(100) # full duty cycle
time.sleep(1)
pwm.change_duty_cycle(50)
time.sleep(200)
pwm.change_frequency(25_000)

pwm.stop()
