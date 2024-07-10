import time
import constants
from lib.mc.controller import GPIO, MotorController
from lib.mc.motor import Motor

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)

    motor = Motor(1)

    # move each motor up and down with utils to test
    motor.set(20) # pos -> down, neg -> up

    t_end = time.time() + 3
    while time.time() < t_end:
        print(GPIO.input(constants.encoder_pins[1]))

    motor.stop()

    # mc._calibrate_abs_enc_positions()


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