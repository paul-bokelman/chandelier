import asyncio
import constants
import RPi.GPIO as GPIO
from lib.mc.controller import MotorController
from lib.sequences import Sequences
from constants import n_motors

async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)
    await mc.calibrate(reset=False)
    await mc.move_all_home()

    for i in range(10):
        await mc.move_all([i / 2 for c in range(n_motors)], i / 2)


    mc.stop_all_motors()
    # mc.sequence(Sequences.WAVE)

    mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())