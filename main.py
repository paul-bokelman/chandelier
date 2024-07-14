import asyncio
import time
import constants
import RPi.GPIO as GPIO
from lib.mc.controller import MotorController
from lib.sequences import RandomSequence

async def idle_state(mc: MotorController):
    """Idle state"""
    await mc.move_all_home()
    mc.stop_all_motors()
    
async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)

    await  mc.move_all_home()

    sequence = RandomSequence(mc)

    await sequence.start()

    time.sleep(30)

    sequence.stop()

    # await mc.calibrate(reset=True)

    # await mc.move_all([0.5, 0.5, 0.5, 0.5], 0.5)
    # await mc.move_all_home()
    
    mc.stop_all_motors()

    mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())