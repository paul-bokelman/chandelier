import asyncio
import RPi.GPIO as GPIO
import constants
from lib.state import StateMachine
from lib.controller import MotorController
from lib.utils import log
import argparse

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

        # blank test mode
        if args.mode == "blank":
            log.info("Running blank test mode")
            mc = MotorController()
            asyncio.run(mc.move_all_home(-0.2))
            asyncio.run(mc.move_all(0.1, 0.5))
            asyncio.run(mc.move_all_home(-0.2))
            mc.stop_all_motors()
            log.info("Blank test mode complete")
        # encoders test mode
        elif args.mode == "encoders":
            log.info("Running encoders test mode")
            mc = MotorController()
            asyncio.run(mc.move_all_home(-0.2))
            asyncio.run(mc.move_all(0.1, 0.5))
            asyncio.run(mc.move_all_home(-0.2))
            mc.stop_all_motors()

            disabled_motors = [m for m in mc.motors if m.disabled]

            if len(disabled_motors) > 0:
                log.error(f"Disabled motors: {disabled_motors}")
            else:
                log.info("All motors are enabled")
        # normal operation mode
        else:
            sm = StateMachine()

            # run state machine forever
            while True:
                asyncio.run(sm.check())

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