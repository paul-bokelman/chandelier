from MC import GPIO, initialize, home_motors, move_motors_counts, stop_all_motors, move_motors_home
import constants

GPIO.setmode(GPIO.BCM)
GPIO.setup(constants.MEGM, GPIO.IN, pull_up_down=GPIO.PUD_UP)

return_val = []

#data_output(1,EnCounts)
#move_motors_home(30)
#move_motor_time(0,20,2)
#move_motors_home(30)
#encoder_speed_calibration()  
initialize()
home_motors()
motor_speeds = [20,0,0,0]
tar_positions = [5,0,0,0]
#move_motors_counts(tar_positions,"",motor_speeds)
return_val = move_motors_counts(tar_positions,"s")
move_motors_home()
stop_all_motors(constants.Num_Motors)