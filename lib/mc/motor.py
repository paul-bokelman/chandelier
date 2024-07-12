from typing import Optional
import time
import RPi.GPIO as GPIO
from PCA9685 import pwm
from lib.store import DataMode
from lib.utils import log, to_pulse
import constants

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, pin: int, is_home: bool = False) -> None:
        self.pin = pin
        self.last_read_time = None
        self.disabled = False
        self.encoder_feedback_disabled = False # get feedback from encoder
        self.direction = constants.down # direction of motor
        self.max_counts = 30
        self.encoder_pin = constants.encoder_pins[self.pin]

        # calibrated data
        self.counts = -1  # current count position, -1 indicates that the motor has not been calibrated
        self.cps_down = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up = None # counts per second moving up, -1 indicates that the motor has not been calibrated

        # todo: pull all calibration data for this motor

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

    def _encoder_callback(self, channel: int):
        """Callback function for encoder"""
        if self.encoder_feedback_disabled:
            return

        self.counts += self.direction * 1 # increment encoder count
        self.last_read_time = time.time()
        log.info(f"M{self.pin} | count: {self.counts} | direction: {'down' if self.direction == constants.down else 'up'}")

    def _at_home(self):
        """Set motor home state"""
        self.encoder_feedback_disabled = True
        self.last_read_time = None
        self.direction = constants.down
        self.counts = 0

    def _error(self, message: str):
        """Set motor error state"""
        self.encoder_feedback_disabled = True
        self.counts = 0
        self.disabled = True
        log.error(message)
        self.stop()

    def _save_current_state(self):
        """Save the current state of the motor"""
        pass

    def set(self, speed: float, direction: int = constants.down):
        """Set a specific motor to a specific speed, speed is a value between -1 and 1"""
        pwm.setServoPulse(self.pin, to_pulse(speed, direction))

    def stop(self):
        """Stop the motor"""
        pwm.setServoPulse(self.pin, constants.stop_pulse) 

    def to_home(self, speed: float = constants.to_home_speed):
        """Move the motor to the home position (0 count)"""
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        if self.is_home(): 
            log.success(f"Motor {self.pin} at home")
            return
        if self.disabled: 
            self._error(f"Motor {self.pin} is disabled, cannot move to home")
            return

        self.direction = constants.up
        self.set(speed, self.direction)
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

    async def to(self, target: float, speed: float):
        """Move the motor to a specific position in counts, target is a percentage of the max counts"""
        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        target_counts = int((target / 1 ) * (self.max_counts))

        log.info(f'Moving: M{self.pin} ({self.counts} -> {target_counts}) at speed {speed}')

        # change direction based on target position
        if target_counts > self.counts:
            self.direction = constants.down
        else:
            self.direction = constants.up

        if self.counts == target_counts:
            log.success(f"Motor {self.pin} already at target position")
            return
        
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        self.set(speed, self.direction) # set the motor in the correct direction

        start_time = time.time()
        while True:
            # check if the motor has reached the target position
            if self.counts == target_counts:
                log.success(f"Motor {self.pin} has reached target position")
                break
            # motor has timed out -> stop the motor
            if time.time() - start_time > constants.to_position_timeout:
                self._error(f"Motor {self.pin} timed out moving to target position, disabling...")
                break
        
        self.encoder_feedback_disabled = True # stop incrementing encoder counts
        self.stop()

    def calibrate(self, data: list[Optional[float]] = [None, None, None]):
        """Calibrate the motor to determine lower and upper bounds of motor speed"""

        # check if motor is already calibrated
        if data[DataMode.cps_down.value] is not None and data[DataMode.cps_up.value] is not None:
            log.info(f"Motor {self.pin} already calibrated")
            self.cps_down = data[DataMode.cps_down.value]
            self.cps_up = data[DataMode.cps_up.value]

            log.success(f"M{self.pin} | cps up: {self.cps_up} | cps down: {self.cps_down}")
            return

        # ensure motor is at home before calibrating
        if not self.is_home():
            self.to_home()

        # disabled -> cannot calibrate
        if self.disabled:
            self._error(f"Motor {self.pin} is disabled, cannot calibrate")
            return
        
        self.encoder_feedback_disabled = False # start incrementing encoder counts

        log.info(f"Calibrating M{self.pin}")

        # get cps down from data if available
        if data[DataMode.cps_down.value] is not None:
            self.cps_down = data[DataMode.cps_down.value]
            log.info(f"Using provided cps down: {self.cps_down}")
        else: # calculate cps down otherwise
            log.info(f"Calculating M{self.pin} cps down")
            self.direction = constants.down 
            self.set(constants.calibration_speed, self.direction) # set the motor to the calibration speed

            start = time.time()
            # move the motor to the calibration position
            while self.counts != constants.calibration_counts:
                # motor has timed out -> error
                if time.time() - start > constants.calibration_timeout:
                    self._error(f"Motor {self.pin} timed out calibrating, disabling...")
                    return
                
            self.stop()
            
            down_time = time.time() - start # time taken to move to calibration position
            self.cps_down = constants.calibration_counts / down_time # time per count

        # get cps up from data if available
        if data[DataMode.cps_up.value] is not None:
            self.cps_up = data[DataMode.cps_up.value]
            log.info(f"Using provided cps up: {self.cps_up}")
        else:
            log.info(f"Calculating M{self.pin} up cps")

            start = time.time()
            self.to_home(speed=constants.calibration_speed)
            
            up_time = time.time() - start
            self.cps_up = constants.calibration_counts / up_time

        log.success(f"M{self.pin} | cps up: {self.cps_up} | cps down: {self.cps_down}")
        
        self.encoder_feedback_disabled = True

    def is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0