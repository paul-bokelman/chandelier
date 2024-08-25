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
n_motors = 16 # number of active motors
debug = True # show debug messages
suppress_count_logging = True # suppress encoder count logging
testing_mode = True # general testing mode (uses constants prefixed with testing_)
mimic_home = False # mimic home position, used when candles aren't present
initial_disabled_motors = [] # list of disabled motors, ex: [1, 2] disables motors at index 1 and 2

candles_per_charge_cycle = 2 # number of candles to charge per charge cycle
charge_cycle_time = 60 * 10 # 10 minutes (in seconds)
available_charging_hours = [0, 1, 2] # charge at 12, 1 and 2 o'clock (24hr)
testing_charge_cycle_time = 10 # 10 seconds
testing_available_charging_hours = [i for i in range(24)] # available charging at every hour (24hr)

max_random_state_time = 60 * 60 * 2 # 2 hours (in seconds)
max_sequence_state_time = 60 * 60 * 2 # 2 hours (in seconds)
testing_max_random_state_time = 60 * 5 # 5 minutes
testing_max_sequence_state_time = 60 * 5 # 5 minutes

# throttle configuration
class ThrottlePresets(Enum):
    """Speed configuration enum, values are neutral throttle offsets"""
    SLOW = 0.04
    MEDIUM = 0.05
    FAST = 0.06

# motor configuration
uncalibrated_home_throttle = -0.2 # speed to move to home
to_home_timeout = 180 # max timeout for trying to move home
to_home_initial_timeout = 3 # initial timeout for trying to move home
to_home_max_interval = 1.3 # seconds between encoder readings (should be relative to speed...)
to_position_timeout = 180 # max timeout for trying to move to a position
max_time_between_encoder_readings = 6 # max time (in seconds) between encoder readings before disabling

calibration_to_position_timeout = 3 # max timeout for trying to move to a position during calibration
calibration_counts = 8 # distance to move for calibration (in counts)
calibration_timeout = 30 # max timeout for calibration
calibration_speed_step = 0.01 # speed step for min speed calibration
calibration_total_steps = 30 # total steps for min speed calibration (0.30->0)
calibration_speed = 0.2 # speed for calibration

calibrations_file_path = "calibrations.json"

max_counts = 90 # max encoder counts