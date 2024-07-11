from typing import Optional
import time
import RPi.GPIO as GPIO
from PCA9685 import pwm
from lib.utils import log, to_pulse
import constants

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, pin: int, is_home: bool = False) -> None:
        self.pin = pin
        self.home = is_home
        self.last_read_time = None
        self.disabled = False
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
        # if self.count_position < 0:
        #     self._error(f"Motor {self.pin} encoder count is negative, disabling...")
        #     return

        self.count_position += self.direction * 1 # increment encoder count
        self.last_read_time = time.time()
        log.info(f"Motor {self.pin} encoder count: {self.count_position}, time: {self.last_read_time}")

    def _at_home(self):
        """Set motor home state"""
        self.home = True
        self.prev_encoder_reading = None
        self.direction = constants.down
        self.count_position = 0

    def _error(self, message: str):
        """Set motor error state"""
        self.home = False
        self.prev_encoder_reading = None
        self.count_position = 0
        self.disabled = True
        log.error(message)
        self.stop()

    def set(self, speed: float):
        """Set a specific motor to a specific speed"""
        pwm.setServoPulse(self.pin, to_pulse(speed))

    def stop(self):
        """Stop the motor"""
        pwm.setServoPulse(self.pin, constants.stop_pulse) 

    def to_home(self, speed: float = constants.to_home_speed):
        """Move the motor to the home position"""
        if self.home: return
        if self.disabled: 
            self._error(f"Motor {self.pin} is disabled, cannot move to home")
            return

        self.direction = constants.up
        self.set(self.direction * speed)
        self.last_read_time = None # reset last read time
        start_time = time.time()
        log.info(f"Moving motor {self.pin} to home")
        while True:
            current_time = time.time()
            # initial count position has not changed -> already home or jammed
            if current_time - start_time > constants.to_home_initial_timeout and self.last_read_time is None:
                log.success(f"Motor {self.pin} already at home")
                self._at_home()
                break
            # check if the motor has reached home
            if self.last_read_time is not None and current_time - self.last_read_time > constants.to_home_max_interval:
                log.success(f"Motor {self.pin} has reached home")
                self._at_home()
                break
            # if the motor has timed out, stop the motor
            if current_time - start_time > constants.to_home_timeout:
                self._error(f"Motor {self.pin} timed out")
                return
        
        self.stop() # stop the motor

    def to(self, target: float, speed: float): # value between 0 and 1
        """Move the motor to a specific position in counts"""

        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")

        target_counts = int((target / 1 ) * (self.max_counts))

        log.info(f'Moving motor {self.pin} from {self.count_position} to target position {target_counts}')

        # change direction based on target position
        if target_counts > self.count_position:
            self.direction = constants.down
        else:
            self.direction = constants.up

        self.set(self.direction * speed) # set the motor in the correct direction

        start_time = time.time()
        while True:
            # check if the motor has reached the target position
            if self.count_position == target_counts:
                log.success(f"Motor {self.pin} has reached target position")
                break
            # motor has timed out -> stop the motor
            if time.time() - start_time > constants.to_position_timeout:
                self._error(f"Motor {self.pin} timed out moving to target position, disabling...")
                break

        self.stop()

    def calibrate(self):
        """Calibrate the motor to determine lower and upper bounds of motor speed"""
        # ensure motor is at home 

        if not self.home:
            self.to_home()

        if self.disabled:
            self._error(f"Motor {self.pin} is disabled, cannot calibrate")
            return
        
        # measure slowest speed down

        self.direction = constants.down
        self.set(self.direction * constants.calibration_speed) # set the motor to the calibration speed


        # measure slowest speed up
        
        # # measure down counts
        # self.direction = constants.down 
        # self.set(self.direction * constants.calibration_speed) # set the motor to the calibration speed

        # start = time.time()
        # # move the motor to the calibration position
        # while self.count_position != constants.calibration_counts:
        #     # motor has timed out -> error
        #     if time.time() - start > constants.calibration_timeout:
        #         self._error(f"Motor {self.pin} timed out calibrating, disabling...")
        #         return
        
        # down_time = time.time() - start # time taken to move to calibration position
        # cps = constants.calibration_counts / down_time # time per count
        

        # self.stop() # stop the motor
            

    def is_home(self):
        """Returns whether the motor is at home or not"""
        return self.home
        