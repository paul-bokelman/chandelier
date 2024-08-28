import asyncio
import argparse
import RPi.GPIO as GPIO
import constants
from modes import normal, blank, encoders, auto
from lib.state import StateMachine
from lib.controller import MotorController
from lib.utils import log

def main():
    parser = argparse.ArgumentParser(description="chandelier")
    parser.add_argument("-m", "--mode", help="Select mode for chandelier", type=str, default='normal')
    args = parser.parse_args()

    if args.mode not in constants.modes:
        log.error(f"Invalid mode: {args.mode}, available modes: {constants.modes}")
        return

    try: 
        # set up GPIO pins
        GPIO.cleanup() # clean up any existing GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.wall_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.led_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.charging_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(constants.reboot_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        if args.mode == "blank":
            blank.blank_mode()
        elif args.mode == "encoders":
            encoders.encoders_mode()
        elif args.mode == "auto":
            auto.auto_mode()
        else:
            normal.normal_mode()

    except Exception as e:
        log.error(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    try:
       main()
    except KeyboardInterrupt:
        log.warning("Interrupted by user")
        GPIO.cleanup()
        exit()
    except Exception as e:
        log.error(f"An error occurred: {e}")
        GPIO.cleanup()
        exit()