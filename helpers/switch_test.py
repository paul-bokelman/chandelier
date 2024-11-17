import time
import asyncio
from lib.utils import log
from configuration.config import config
import argparse
import sshkeyboard
from configuration.config import Config, Environments, config
from helpers.stop import emergency_stop

try:
    import RPi.GPIO as GPIO # type: ignore
except ImportError:
    import Mock.GPIO as GPIO

def main():

    try: 
        # set up GPIO pins
        GPIO.cleanup()  # clean up any existing GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(config.get('service_button_pin'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.get('wall_switch_pins'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(config.get('led_pin'), GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(config.get('reboot_button_pin'), GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        while True:
            if GPIO.input('service_button_pin') == GPIO.HIGH:
                log.info("Switch",message="Service button pressed")
            elif GPIO.input('reboot_button_pin') == GPIO.HIGH:
                log.info("Switch",message="Reboot button pressed")
            else:
                log.info("Switch",message="Nothing")
            
            if GPIO.input('led_pin') == GPIO.LOW:
                GPIO.output('led_pin', GPIO.HIGH)
            else:
                GPIO.output('led_pin', GPIO.LOW)

            time.sleep(1)

    except Exception as e:
        log.error(f"An error occurred: {e}")
    finally:
        asyncio.run(emergency_stop()) # run emergency stop
        sshkeyboard.stop_listening() # stop listening for ssh keyboard input
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
    finally:
        GPIO.cleanup()
        sshkeyboard.stop_listening() # stop listening for ssh keyboard input
        exit()