import asyncio
import RPi.GPIO as GPIO
import constants
from lib.controller import MotorController
from lib.utils import log

async def blank_test():
    try: 
        # set up GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.wall_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.led_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.charging_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.reboot_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        mc = MotorController()

        await mc.move_all(0.9, 0.5)

        mc.stop_all_motors()

    except Exception as e:
        log.error(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

asyncio.run(blank_test())