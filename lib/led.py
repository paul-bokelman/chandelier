import RPi.GPIO as GPIO
import asyncio

class LED:
    def __init__(self, pin):
        self.pin = pin
        self.state = False
        self.off()

    def on(self):
        self.state = True
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        self.state = False
        GPIO.output(self.pin, GPIO.LOW)

    async def blink(self, duration):
        self.on()
        await asyncio.sleep(duration)
        self.off()
        