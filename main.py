import asyncio
import time
import constants
import RPi.GPIO as GPIO
# from lib.mc.controller import MotorController
# from lib.sequence import Sequence

# async def idle_state(mc: MotorController):
#     """Idle state"""
#     await mc.move_all_home()
#     mc.stop_all_motors()
    
async def main():
    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    # GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(ledpin, GPIO.OUT) # set ledpin as an output
    GPIO.setup(constants.led_pin, GPIO.OUT) # set ledpin as an output
    GPIO.setup(constants.service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(constants.reboot_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(constants.wall_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # turn on led

    def button_callback(channel):
        print(f"Button {channel} was pushed!")

    GPIO.add_event_detect(constants.service_button_pin, GPIO.FALLING, callback=button_callback, bouncetime=300)

    GPIO.add_event_detect(constants.reboot_button_pin, GPIO.FALLING, callback=button_callback, bouncetime=300)

    GPIO.add_event_detect(constants.wall_switch_pins[0], GPIO.FALLING, callback=button_callback, bouncetime=300)

    GPIO.add_event_detect(constants.wall_switch_pins[1], GPIO.FALLING, callback=button_callback, bouncetime=300)


    while True:
        GPIO.output(constants.led_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(constants.led_pin, GPIO.LOW)
        time.sleep(0.5)



    # mc = MotorController(debug=True)
    # await mc.move_all(0.5)

    # await mc.move_all_home()

    # run random sequence
    # sequence = Sequence()

    # for positions, speed in sequence.random():
    #     await mc.move_all(positions, speed)

    # await mc.calibrate(reset=True)

    # await mc.move_all([0.5, 0.5, 0.5, 0.5], 0.5)
    # await mc.move_all_home()
    
    # mc.stop_all_motors()

    # mc.save_calibration()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())