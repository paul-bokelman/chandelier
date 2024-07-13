encoder_pins = [14,15,18,4,26,17,27,22,19,10,9,11,13,0,5,6] #Motor Encoder GPIO Mapping
WSI = [23,24,25,8] #Wall Switch Input
PS = [16,20] # Power Switch
n_motors = 4
Delta_Slow_Med_Speed = 5
Delta_Slow_Fast_Speed = 10
stop_pulse = 1500

# debug mode
debug = False

# static
up = -1
down = 1
fast_speed = 1
mid_speed = 0.5
slow_speed = 0.25

# motor configuration
to_home_speed = 0.08 # speed to move to home
to_home_timeout = 120 # max timeout for trying to move home
to_home_initial_timeout = 2.5 # initial timeout for trying to move home
to_home_max_interval = 1.5 # seconds between encoder readings (should be relative to speed...)
to_position_timeout = 60 # max timeout for trying to move to a position
calibration_counts = 20 # distance to move for calibration (in counts)
calibration_timeout = 30 # max timeout for calibration
calibration_speed_step = 0.05 # speed step for min speed calibration
calibration_total_steps = 12 # total steps for min speed calibration (0.55->0)
calibration_speed = 0.5 # speed for calibration

calibrations_file_path = "calibrations.json"

# sequence configuration
# (max_speed + min_speed) / 2 = calibration_speed
max_speed = 20 # relative to calibration speed
min_speed = 2 # relative to calibration speed 
max_counts = 30 # max encoder counts