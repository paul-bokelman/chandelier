from enum import Enum

# static configuration
encoder_pins = [14,15,18,4,26,17,27,22,19,10,9,11,13,0,5,6] #Motor Encoder GPIO Mapping
power_switch_pins = [16,20] # Power Switch
wall_switch_pins = [24, 25]
service_button_pin = 23
reboot_button_pin = 7
led_pin = 8
up = -1
down = 1


# general configuration
n_motors = 4
debug = False
testing_mode = True
disabled_motors = [0, 1]
n_active_motors = n_motors - len(disabled_motors)

max_charge_time = 7200 # 2 hours
charge_interval = 86400 * 5 # 5 days
max_run_time = 7200 # 2 hours

# throttle configuration
class ThrottlePresets(Enum):
    """Speed configuration enum, values are neutral throttle offsets"""
    SLOW = 0.01
    MEDIUM = 0.02
    FAST = 0.03

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