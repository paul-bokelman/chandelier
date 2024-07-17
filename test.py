import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

kit.continuous_servo[2].throttle = 1
time.sleep(1)
kit.continuous_servo[2].throttle = -1
time.sleep(1)
kit.continuous_servo[2].set_pulse_width_range(1500, 2000)
kit.continuous_servo[2].throttle = 0