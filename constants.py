from enum import Enum

encoder_pins = [14,15,18,4,26,17,27,22,19,10,9,11,13,0,5,6] #Motor Encoder GPIO Mapping
power_switch_pins = [16,20] # Power Switch

wall_switch_pins = [24, 25]
service_button_pin = 23
reboot_button_pin = 7
led_pin = 8

n_motors = 4
disabled_motors = [0, 1]
Delta_Slow_Med_Speed = 5
Delta_Slow_Fast_Speed = 10
stop_pulse = 1500

# debug mode
debug = False

# static
up = -1
down = 1

# throttle configuration
class ThrottlePresets(Enum):
    """Speed configuration enum, values are neutral throttle offsets"""
    SLOW = 0.1
    MEDIUM = 0.2
    FAST = 0.3

# motor configuration
uncalibrated_home_throttle = 0.0 # speed to move to home
to_home_timeout = 120 # max timeout for trying to move home
to_home_initial_timeout = 5 # initial timeout for trying to move home
to_home_max_interval = 2.5 # seconds between encoder readings (should be relative to speed...)
to_position_timeout = 60 # max timeout for trying to move to a position

calibration_to_position_timeout = 3 # max timeout for trying to move to a position during calibration
calibration_counts = 10 # distance to move for calibration (in counts)
calibration_timeout = 30 # max timeout for calibration
calibration_speed_step = 0.01 # speed step for min speed calibration
calibration_total_steps = 30 # total steps for min speed calibration (0.30->0)
calibration_speed = 0.2 # speed for calibration

calibrations_file_path = "calibrations.json"

# sequence configuration
# (max_speed + min_speed) / 2 = calibration_speed
max_speed = 32 # relative to calibration speed
min_speed = 0 # relative to calibration speed 
max_counts = 30 # max encoder counts

# charging
max_charge_time = 7200 # 2 hours
charge_interval = 86400 * 5 # 5 days

# sequences
max_run_time = 7200 # 2 hours
