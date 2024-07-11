from typing import Optional
import time
import RPi.GPIO as GPIO
from PCA9685 import pwm
from lib.utils import to_pulse
import constants

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, pin: int, is_home: bool = False) -> None:
        self.pin = pin
        self.home = is_home
        self.last_read_time = None
        self.count_position = 0
        self.direction = constants.down # direction of motor
        self.max_counts = 30
        self.encoder_pin = constants.encoder_pins[self.pin]
        self.prev_encoder_reading: Optional[int] = None

        # todo: pull all calibration data for this motor

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

    def _encoder_callback(self, channel: int):
        """Callback function for encoder"""
        self.count_position += self.direction * 1 # increment encoder count
        self.last_read_time = time.time()
        print(f"Motor {self.pin} encoder count: {self.count_position}, time: {self.last_read_time}")

    def set(self, speed: float):
        """Set a specific motor to a specific speed"""
        pwm.setServoPulse(self.pin, to_pulse(speed))

    def stop(self):
        """Stop the motor"""
        pwm.setServoPulse(self.pin, constants.stop_pulse) 

    def to_home(self):
        """Move the motor to the home position"""
        if self.home: return
        self.direction = constants.up
        self.set(constants.to_home_speed)
        self.last_read_time = None # reset last read time
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
        
        self.count_position = 0 #? should decrement in encoder callback...
        self.stop() # stop the motor

    def to(self, target: float, speed: float): # value between 0 and 1
        """Move the motor to a specific position in counts"""

        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")

        target_counts = int((target / 1 ) * (self.max_counts))

        print(f'Moving motor {self.pin} from {self.count_position} to target position {target_counts}')

        if target_counts > self.count_position:
            self.direction = constants.down

        self.set(self.direction * speed)

        start_time = time.time()
        while True:
            if self.count_position == target_counts:
                print(f"Motor {self.pin} has reached target position")
                break
            if time.time() - start_time > constants.to_position_timeout:
                print(f"Motor {self.pin} timed out moving to target position")
                break

        self.stop()

    def calibrate(self):
        """Calibrate the motor by moving it to the home position then calculating the max counts"""
        self.to_home() # move the motor to the home position

        # calculate the max counts

        pass

    def is_home(self):
        """Returns whether the motor is at home or not"""
        return self.home
        