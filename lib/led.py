import asyncio
try:
    import RPi.GPIO as GPIO # type: ignore
except ImportError:
    import Mock.GPIO as GPIO

class LED:
    def __init__(self, pin):
        self.pin = pin
        self.state = False
        self.blinking = False
        self.off()

    def on(self):
        self.state = True
        self.blinking = False
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        self.state = False
        self.blinking = False
        GPIO.output(self.pin, GPIO.LOW)

    async def blink(self, duration: float = 1.0):
        self.blinking = True
        while self.blinking:
            GPIO.output(self.pin, GPIO.HIGH)
            await asyncio.sleep(duration)
            GPIO.output(self.pin, GPIO.LOW)
            await asyncio.sleep(duration)

    async def double_blink(self, duration: float = 1.0):
        self.blinking = True
        while self.blinking:
            GPIO.output(self.pin, GPIO.HIGH)
            await asyncio.sleep(duration)
            GPIO.output(self.pin, GPIO.LOW)
            await asyncio.sleep(duration)
            GPIO.output(self.pin, GPIO.HIGH)
            await asyncio.sleep(duration)
            GPIO.output(self.pin, GPIO.LOW)
            await asyncio.sleep(duration * 2)