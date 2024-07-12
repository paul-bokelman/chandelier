import asyncio
import constants
import RPi.GPIO as GPIO
from lib.mc.controller import MotorController
from lib.sequences import Sequences

async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)
    await mc.calibrate(reset=False)
    await mc.move_all_home()

    await mc.move_all([0.7, 0.7, 0.7, 0.7], 0.8)
    
    mc.stop_all_motors()

    mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())