from typing import Optional
import time
import RPi.GPIO as GPIO
from PCA9685 import pwm
from lib.utils import to_pulse
import constants

class Motor:
    def __init__(self, pin: int, is_home: bool = False) -> None:
        self.pin = pin
        self.home = is_home
        self.last_read_time = None
        self.encoder_count = 0
        self.encoder_pin = constants.encoder_pins[self.pin]
        self.prev_encoder_reading: Optional[int] = None

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

    def _encoder_callback(self, channel: int):
        """
        Callback function for encoder
        """
        self.encoder_count += 1 # increment encoder count
        self.last_read_time = time.time()
        print(f"Motor {self.pin} encoder count: {self.encoder_count}, time: {self.last_read_time}")

    def set(self, speed: float):
        """
        Set a specific motor to a specific speed
        """
        pwm.setServoPulse(self.pin, to_pulse(speed))

    def stop(self):
        pwm.setServoPulse(self.pin, constants.stop_pulse) 

    def to_home(self):
        """
        Move the motor to the home position
        """
        self.set(constants.to_home_speed)
        start_time = time.time()
        while True:
            # check if the motor has reached home
            if self.last_read_time is not None and time.time() - self.last_read_time > constants.to_home_max_interval:
                print(f"Motor {self.pin} has reached home")
                self.last_read_time = None
                self.home = True
                break
            # if the motor has timed out, stop the motor
            if time.time() - start_time > constants.to_home_timeout:
                print(f"Motor {self.pin} timed out")
                self.home = True 
                break

        self.stop()

    def to(self):
        """
        Move the motor to a specific position
        """
        pass

    def is_home(self):
        """
        Returns whether the motor is at home or not
        """
        return self.home
        