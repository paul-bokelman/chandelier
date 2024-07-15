import asyncio
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
    GPIO.setup(constants.WSI, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    while True: # Run forever
        if GPIO.input(constants.WSI[0]) == GPIO.HIGH:
            print("Wall Switch 1 is pressed")
        if GPIO.input(constants.WSI[1]) == GPIO.HIGH:
            print("Wall Switch 2 is pressed")
        if GPIO.input(constants.WSI[2]) == GPIO.HIGH:
            print("Wall Switch 3 is pressed")
        if GPIO.input(constants.WSI[3]) == GPIO.HIGH:
            print("Wall Switch 4 is pressed")

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