import constants
from lib.mc.controller import GPIO, MotorController

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)
    # mc.set_all_motors(20)
    # time.sleep(4)
    # mc.stop_all_motors()

    mc.move_all_home() # ensure all motors are at home position before moving
    # mc.calibrate()
    mc.move_all([0.1, 0.1, 0.1, 0.1], 0.2) # move all motors to a specific position
    mc.stop_all_motors()


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