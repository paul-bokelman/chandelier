from controller import GPIO, MotorController
import constants
from time import sleep

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.MEGM, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    return_val = []

    mc = MotorController(debug=True)

    
    # move each motor up and down with utils to test
    mc.set_all_motors(20)

    sleep(4)
    mc.stop_all_motors()


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