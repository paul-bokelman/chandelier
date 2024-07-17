import time

class LED:
    def __init__(self, pin):
        self.pin = pin
        self.state = False
        self.off()

    def on(self):
        self.state = True
        print("LED ON")

    def off(self):
        self.state = False
        print("LED OFF")

    def blink(self, duration: float = 0.5):
        self.on()
        time.sleep(duration)
        self.off()