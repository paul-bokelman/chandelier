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
        self.last_count_time = 0
        self.encoder_count = 0
        self.encoder_pin = constants.encoder_pins[self.pin]
        self.prev_encoder_reading: Optional[int] = None

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(constants.encoder_pins[self.encoder_pin], GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

    def _encoder_callback(self, channel):
        """
        Callback function for encoder
        """
        self.encoder_count += 1 # increment encoder count
        current_time = time.time()
        time_diff = current_time - self.last_count_time # calculate time difference since last reading
        self.last_count_time = current_time
        
        # if time difference is greater than the max interval, motor has stalled and is at home
        if time_diff > constants.to_home_max_interval:
            self.home = True

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
            if self.is_home():
                break
            if time.time() - start_time > constants.to_home_timeout:
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
        