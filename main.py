from controller import GPIO, MotorController
import constants

GPIO.setmode(GPIO.BCM)
GPIO.setup(constants.MEGM, GPIO.IN, pull_up_down=GPIO.PUD_UP)

return_val = []

mc = MotorController(debug=True)

mc.home_motors()

motor_speeds = [20,0,0,0]
tar_positions = [5,0,0,0]
mc.encoder_speed_calibration()
#move_motors_counts(tar_positions,"",motor_speeds)
return_val = mc.move_motors_counts(tar_positions,"s")
mc.move_motors_home()
mc.stop_all_motors(constants.Num_Motors)