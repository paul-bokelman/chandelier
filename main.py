import asyncio
import constants
import RPi.GPIO as GPIO
from lib.mc.controller import MotorController
import math

async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    mc = MotorController(debug=True)
    await mc.calibrate(reset=False)
    await mc.move_all_home()

    await mc.move_all([0.7, 0.7, 0.7, 0.7], 0.5)

    # make sine wave sequence with 0.5 speed

    offset= 0.1

    pos = lambda t, offset_m: math.sin(t + offset * offset_m) * 0.75

    for i in range(20):
        await mc.move_all([pos(i, 0), pos(i, 1), pos(i, 1), pos(i, 2)], 0.5)

    
    mc.stop_all_motors()

    mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())