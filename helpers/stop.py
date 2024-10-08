import asyncio
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

async def emergency_stop():
    try: 
        kit = ServoKit(channels=16)

        for i in range(16):
            kit.continuous_servo[i]._pwm_out.duty_cycle = 0

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

asyncio.run(emergency_stop())