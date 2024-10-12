import sys
import argparse
import traceback
import asyncio
from configuration.config import Config, Environments, config
from modes import normal, manual, scripts, testing
from preflight import calibration
from lib.utils import log
from helpers.stop import emergency_stop

# import GPIO library
try:
    import RPi.GPIO as GPIO # type: ignore
except ImportError:
    import Mock.GPIO as GPIO

def main():
    parser = argparse.ArgumentParser(description="chandelier")
    parser.add_argument("-e", "--env", help="Select an environment", type=str, default='development') # development, testing, production
    parser.add_argument("-m", "--mode", help="Select a mode", type=str, default='normal') # normal, manual, scripts, testing
    parser.add_argument("-c", "--calibration", help="Select calibration option", type=str, default='default') # default, prompt 
    args= parser.parse_args()

    # validate mode arg
    available_modes = config.get("modes")
    if args.mode not in available_modes:
        raise ValueError(f"Invalid mode: {args.mode}, available modes: {available_modes}")
    
    # validate environment arg
    available_environments = [env.value for env in Environments]
    if args.env not in available_environments:
        raise ValueError(f"Invalid environment: {args.env}, available environments: {available_environments}")
    
    # validate calibration arg
    available_calibration_options = calibration.calibration_options
    if args.calibration not in available_calibration_options:
        raise ValueError(f"Invalid calibration option: {args.calibration}, available calibration options: {available_calibration_options}")

    try: 
        # set up GPIO pins
        GPIO.cleanup()  # clean up any existing GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(config.get('encoder_pins'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.get('service_button_pin'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.get('wall_switch_pins'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.get('led_pin'), GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(config.get('charging_pin'), GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(config.get('reboot_button_pin'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        config.__new__(Config)
        config.load(env=Environments[args.env.upper()]) # load configuration

        skip_calibration = args.mode == "scripts" # skip calibration for scripts mode (calibration not needed)

        # preflight
        asyncio.run(calibration.preflight(option=args.calibration, skip=skip_calibration))

        # execute mode
        if args.mode == "manual":
            manual.run()
        elif args.mode == "normal":
            normal.run()
        elif args.mode == "scripts":
            scripts.run()
        elif args.mode == "testing":
            testing.run()
        else:
            log.error(f"Invalid mode: {args.mode}")

    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()

        if exc_type is None or exc_value is None or exc_traceback is None:
            log.error(f"An error occurred: {e}")
            return

        tb_str = ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        log.error(f"An error occurred: {e}\n\nFull traceback:\n{tb_str}")
        log.error(f"Exception type: {exc_type.__name__}")
        log.error(f"Exception value: {exc_value}")
        log.error(f"Exception occurred in file: {exc_traceback.tb_frame.f_code.co_filename}")
        log.error(f"Exception occurred on line: {exc_traceback.tb_lineno}")
        log.error(f"Local variables at the point of exception:")
        for key, value in exc_traceback.tb_frame.f_locals.items():
            log.error(f"    {key}: {value}")
    finally:
        asyncio.run(emergency_stop()) # run emergency stop
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