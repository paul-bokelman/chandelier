import time
import asyncio
import RPi.GPIO as GPIO

class LED:
    def __init__(self, pin):
        self.pin = pin
        self.state = False
        self.blinking = False
        self.off()

    def on(self):
        self.state = True
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        self.state = False
        GPIO.output(self.pin, GPIO.LOW)

    async def blink(self, duration: float = 1):
        self.blinking = True
        while self.blinking:
            self.on()
            time.sleep(duration)
            self.off()
            time.sleep(duration)
            await asyncio.sleep(0)

    def stop_blink(self):
        self.blinking = False
        self.off()

    def reset(self):
        self.stop_blink()
        self.off()