import asyncio
import RPi.GPIO as GPIO
import time
import constants
# from lib.mc.controller import MotorController
from lib.state import StateMachine
from lib.utils import log

def test(channel):
    print("Test", channel)

async def main():
    try: 
        # set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.wall_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.led_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.reboot_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        sm = StateMachine()

        GPIO.add_event_detect(constants.wall_switch_pins[0], GPIO.FALLING, callback=test, bouncetime=300)

        while True:
            if not GPIO.input(constants.service_button_pin):
                print("Service button pressed")
            if not GPIO.input(constants.reboot_button_pin):
                print("Reboot button pressed")
            if not GPIO.input(constants.wall_switch_pins[0]):
                print("Wall switch 0 pressed")
            if not GPIO.input(constants.wall_switch_pins[1]):
                print("Wall switch 1 pressed")
            GPIO.output(constants.led_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(constants.led_pin, GPIO.LOW)

        # while True:
        #     await sm.check()

    # await sm.check()

    # mc = MotorController()
    # await mc.calibrate(reset=False)

    # await mc.move_all(0.5)
    # await mc.move_all_home()

    # mc.stop_all_motors()

    # GPIO.cleanup() # clean up for next session

    except Exception as e:
        log.error(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.warning("Interrupted by user")
        GPIO.cleanup()
        exit()