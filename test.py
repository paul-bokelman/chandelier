import time
from adafruit_servokit import ServoKit

# Initialize the ServoKit instance for a PCA9685 board with 16 channels
kit = ServoKit(channels=16)

# Define the servo channel (e.g., channel 0)
servo_channel = 2

# Function to find the neutral position
def find_neutral(kit, channel):
    step = 0.01
    throttle = 0
    while throttle <= 1:
        kit.continuous_servo[channel].throttle = throttle
        time.sleep(1)
        print(f"Throttle {throttle}: Does it stop? (Press Ctrl+C to stop and try next value)")
        throttle += step
    throttle = 0
    while throttle >= -1:
        kit.continuous_servo[channel].throttle = throttle
        time.sleep(1)
        print(f"Throttle {throttle}: Does it stop? (Press Ctrl+C to stop and try next value)")
        throttle -= step
    kit.continuous_servo[channel].throttle = 0

# Start calibration
try:
    find_neutral(kit, servo_channel)
except KeyboardInterrupt:
    print("Calibration stopped.")

# Set the servo to a neutral position (stop)
kit.continuous_servo[servo_channel].throttle = 0