import asyncio
import time
import schedule
import RPi.GPIO as GPIO
import constants
from lib.mc.controller import MotorController
from lib.sequence import Sequence

# async def idle_state(mc: MotorController):
#     """Idle state"""
#     await mc.move_all_home()
#     mc.stop_all_motors()

    
async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)
    await mc.move_all_home()

    # run random sequence
    sequence = Sequence()

    # for positions, speed in sequence.alternating():
    #     await mc.move_all(positions, speed)

    # await mc.calibrate(reset=True)

    mc.stop_all_motors()

    mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())