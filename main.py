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
    mc.move_all_home()
    mc.calibrate(reset=False)

    move = asyncio.create_task(mc.move_all([0.7, 0.7, 0.7, 0.7], 0.13))
    await move
    mc.stop_all_motors()
    # mc.sequence(Sequences.WAVE)

    mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())