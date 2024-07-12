import constants
from lib.mc.controller import GPIO, MotorController

def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)

    mc.save_calibration()
    mc.calibrate(reset=True)

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    main()