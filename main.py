import asyncio
import time
import constants
import RPi.GPIO as GPIO
# from lib.mc.controller import MotorController
# from lib.sequence import Sequence

# async def idle_state(mc: MotorController):
#     """Idle state"""
#     await mc.move_all_home()
#     mc.stop_all_motors()
    
async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(ledpin, GPIO.OUT) # set ledpin as an output
    GPIO.setup(constants.WSI[0], GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # with pull up resistor

    while True:
        # GPIO.output(ledpin, GPIO.input(pushpin))
        if GPIO.input(constants.WSI[0]) == GPIO.HIGH:
            print('pressed')


    # mc = MotorController(debug=True)
    # await mc.move_all(0.5)

    # await mc.move_all_home()

    # run random sequence
    # sequence = Sequence()

    # for positions, speed in sequence.random():
    #     await mc.move_all(positions, speed)

    # await mc.calibrate(reset=True)

    # await mc.move_all([0.5, 0.5, 0.5, 0.5], 0.5)
    # await mc.move_all_home()
    
    # mc.stop_all_motors()

    # mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())