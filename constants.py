from enum import Enum

# static configuration
encoder_pins = [14,15,18,4,26,17,27,22,19,10,9,11,13,0,5,6] # encoder pins
wall_switch_pins = [24, 25] # 00 -> idle, 10 -> sequence, 01 -> random
service_button_pin = 23
reboot_button_pin = 7
charging_pin = 12
led_pin = 8
up = -1
down = 1

# general configuration
n_motors = 4
debug = True
testing_mode = True
disabled_motors = [0, 1]
n_active_motors = n_motors - len(disabled_motors)

max_charge_time = 60 * 60 * 2 # 2 hours (in seconds)
charge_interval = 60 * 60 * 24 * 5 # 5 days (in seconds)
testing_max_charge_time = 5 # 5 seconds
testing_charge_interval = 20 # 20 seconds
max_run_time = 60 * 60 * 2 # 2 hours (in seconds)

# throttle configuration
class ThrottlePresets(Enum):
    """Speed configuration enum, values are neutral throttle offsets"""
    SLOW = 0.03
    MEDIUM = 0.04
    FAST = 0.05

# motor configuration
uncalibrated_home_throttle = 0.0 # speed to move to home
to_home_timeout = 120 # max timeout for trying to move home
to_home_initial_timeout = 3 # initial timeout for trying to move home
to_home_max_interval = 1.3 # seconds between encoder readings (should be relative to speed...)
to_position_timeout = 60 # max timeout for trying to move to a position

calibration_to_position_timeout = 3 # max timeout for trying to move to a position during calibration
calibration_counts = 8 # distance to move for calibration (in counts)
calibration_timeout = 30 # max timeout for calibration
calibration_speed_step = 0.01 # speed step for min speed calibration
calibration_total_steps = 30 # total steps for min speed calibration (0.30->0)
calibration_speed = 0.2 # speed for calibration

calibrations_file_path = "calibrations.json"

max_counts = 30 # max encoder counts