import asyncio
import RPi.GPIO as GPIO
import constants
# from lib.mc.controller import MotorController
# from lib.state import StateMachine


async def main():
    try: 
        # set up GPIO
        GPIO.setmode(GPIO.BCM)
        input_pins = constants.encoder_pins + constants.wall_switch_pins + [constants.service_button_pin, constants.reboot_button_pin]
        
        GPIO.setup(input_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(constants.led_pin, GPIO.OUT, initial=GPIO.LOW)

    # sm = StateMachine()

    # await sm.check()

    # mc = MotorController()
    # await mc.calibrate(reset=False)

    # await mc.move_all(0.5)
    # await mc.move_all_home()

    # mc.stop_all_motors()

    # GPIO.cleanup() # clean up for next session

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user")
        GPIO.cleanup()
        exit()