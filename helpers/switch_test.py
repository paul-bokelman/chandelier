#Helper script to test buttons, selector swtich and status LED

import time
#from configuration.config import config

try:
    import RPi.GPIO as GPIO # type: ignore
except ImportError:
    import Mock.GPIO as GPIO

def main():

    service_button_pin = 23
    reboot_button_pin = 7
    led_pin = 8
    wall_switch_pins = [24,25]

    try:
        # set up GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(wall_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(reboot_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        while True:
            if not GPIO.input(service_button_pin) == GPIO.HIGH:
                print("Service button pressed")
            elif not GPIO.input(reboot_button_pin) == GPIO.HIGH:
                print("Reboot button pressed")
            elif not GPIO.input(wall_switch_pins[0]) == GPIO.HIGH:
                print("Random mode")
            elif not GPIO.input(wall_switch_pins[1]) == GPIO.HIGH:
                print("Sequence mode")
            else:
                print("Idle State")

            if GPIO.input(led_pin) == GPIO.LOW:
                GPIO.output(led_pin, GPIO.HIGH)
            else:
                GPIO.output(led_pin, GPIO.LOW)

            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    try:
       main()
    except Exception as e:
        log.error(f"An error occurred: {e}")
        GPIO.cleanup()
        exit()
    finally:
        GPIO.cleanup()
        sshkeyboard.stop_listening() # stop listening for ssh keyboard input
        exit()