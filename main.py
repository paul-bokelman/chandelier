import argparse
import RPi.GPIO as GPIO
from configuration.config import Config, Environments, config
from modes import normal, manual
from lib.utils import log

#todo: integrate command line ui for selecting mode of operation (helpers, general) and sub-modes (calibration, normal, auto, etc)
# todo: all modes should be necessary pre-flights and (post-flights?)   

def main():
    parser = argparse.ArgumentParser(description="chandelier")
    parser.add_argument("-e", "--env", help="Select an environment", type=str, default='development')
    parser.add_argument("-m", "--mode", help="Select a mode", type=str, default='normal')
    args = parser.parse_args()

    available_modes = config.get("modes")
    if args.mode not in available_modes:
        log.error(f"Invalid mode: {args.mode}, available modes: {available_modes}")
        return
    
    available_environments = [env.value for env in Environments]
    if args.env not in [env.value for env in Environments]:
        log.error(f"Invalid environment: {args.env}, available environments: {available_environments}")
        return

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

        if args.mode == "auto":
            manual.run()
        elif args.mode == "normal":
            normal.run()
        else:
            log.error(f"Invalid mode: {args.mode}")

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