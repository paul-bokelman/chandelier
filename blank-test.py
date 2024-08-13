import asyncio
import RPi.GPIO as GPIO
import constants
from lib.controller import MotorController
from lib.utils import log

async def blank_test():
    mc = MotorController()
    try: 
        # set up GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.wall_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.led_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.charging_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.reboot_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        await mc.move_all(0.9, 0.5)
        await mc.move_all_home(-0.2)

        mc.stop_all_motors()

    except Exception as e:
        mc.stop_all_motors()
        log.error(f"An error occurred: {e}")
    finally:
        mc.stop_all_motors()
        GPIO.cleanup()

asyncio.run(blank_test())