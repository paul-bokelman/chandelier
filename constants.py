encoder_pins = [14,15,18,4,26,17,27,22,19,10,9,11,13,0,5,6] #Motor Encoder GPIO Mapping
WSI = [23,24,25,8] #Wall Switch Input
PS = [16,20] # Power Switch
n_motors = 4
Delta_Slow_Med_Speed = 5
Delta_Slow_Fast_Speed = 10
Max_Speed = 20
Up_Dir_CCW = False
stop_pulse = 1500

# static
up = 1
down = -1
fast_speed = 1
medium_speed = 0.5
slow_speed = 0.25

# motor configuration
to_home_speed = -5 # speed to move to home
to_home_timeout = 60 # max timeout for trying to move home
to_home_max_interval = 2.5 # seconds between encoder readings (should be relative to speed...)
to_position_timeout = 60 # max timeout for trying to move to a position

calibration_dir = "./data"

