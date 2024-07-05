from MC import *
from PCA9685 import PCA9685

pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)
GPIO.setmode(GPIO.BCM)
GPIO.setup(constants.MEGM, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def initialize():
  #gather data from saved files
  global Encounts, SlowSpeedDown, SlowSpeedUp, MedSpeedDown, MedSpeedUp, FastSpeedDown, FastSpeedDown
  global EncTimeBetCountsSlowDown, EncTimeBetCountsSlowUp,EncTimeBetCountsMedDown, EncTimeBetCountsMedUp, EncTimeBetCountsFastDown, EncTimeBetCountsFastUp
  
  EnCounts = data_input(1)
  SlowSpeedDown = data_input(2)
  SlowSpeedUp = data_input(3)
  MedSpeedDown = data_input(4)
  MedSpeedDown = data_input(5)
  FastSpeedDown = data_input(6)
  FastSpeedUp = data_input(7)
  EncTimeBetCountsSlowDown = data_input(8)
  EncTimeBetCountsSlowUp = data_input(9)
  EncTimeBetCountsMedDown = data_input(10)
  EncTimeBetCountsMedUp = data_input(11)
  EncTimeBetCountsFastDown = data_input(12)
  EncTimeBetCountsFastUp = data_input(13)

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