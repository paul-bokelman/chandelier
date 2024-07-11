import time
import constants
from lib.mc.controller import GPIO, MotorController
from lib.mc.motor import Motor

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)
    # mc.set_all_motors(20)
    # time.sleep(4)
    # mc.stop_all_motors()

    mc.move_all([0.5, 0.25, 0.66, 0.1], 20)

    # mc.move_all_home()

    # motor = Motor(1)

    # # move each motor up and down with utils to test
    # motor.set(20) # pos -> down, neg -> up

    # time.sleep(3)
    # motor.stop()

    # move each motor up and down with utils to test
    # mc.home_motors()

    # todo: infinite loop when running without values in calibration data files

    # motor_speeds = [20,0,0,0]
    # tar_positions = [5,0,0,0]
    ## mc.encoder_speed_calibration()
    ## move_motors_counts(tar_positions,"",motor_speeds)
    # return_val = mc.move_motors_counts(tar_positions,"s")
    # mc.move_motors_home()
    # mc.stop_all_motors(constants.Num_Motors)

    GPIO.cleanup()

if __name__ == "__main__":
    main()