from typing import Optional
from adafruit_servokit import ContinuousServo
import time
import asyncio
import RPi.GPIO as GPIO
from PCA9685 import pwm
from lib.store import DataMode
from lib.utils import log, to_pulse, seconds_elapsed
import constants

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, pin: int, servo: ContinuousServo) -> None:
        self.pin = pin
        self.last_read_time = None
        self.encoder_feedback_disabled = False # get feedback from encoder
        self.direction = constants.down # direction of motor
        self.encoder_pin = constants.encoder_pins[self.pin]
        self.min_down_speed:Optional[float] = None
        self.min_up_speed: Optional[float] = None
        self.servo = servo

        # calibrated data
        # todo: counts should be none unless calibrated (position unknown)
        self.counts = -1  # current count position, -1 indicates that the motor has not been calibrated
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.up_boost: Optional[float] = None # percentage boost needed relative to others, measured at (calibrated counts @ calibrated speed)
        self.down_boost: Optional[float] = None # percentage boost needed relative to others, measured at (calibrated counts @ calibrated speed)

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
        log.error(message)
        self.stop()

    def set(self, speed: float, direction: int = constants.down):
        """Set a specific motor to a specific speed, speed is a value between 0 and 1"""
        assert speed >= 0 and speed <= 1, "Speed must be between 0 and 1"
        assert direction in [constants.up, constants.down], "Direction must be up or down"

        self.servo.throttle = direction * speed

        # pwm.setServoPulse(self.pin, to_pulse(speed, direction, self.up_boost, self.down_boost))

    def stop(self):
        """Stop the motor"""
        log.info(f"Stopping M{self.pin}")
        pwm.setServoPulse(self.pin, constants.stop_pulse) 

    async def to_home(self, speed: float = constants.to_home_speed, override_initial_timeout = False) -> tuple[int, bool]:
        """Move the motor to the home position (0 count)"""
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        timed_out = False
        if self.is_home(): 
            log.success(f"Motor {self.pin} at home")
            return self.counts, timed_out

        self.direction = constants.up
        self.set(speed, self.direction)
        self.last_read_time = None 
        start_time = time.time()
        log.info(f"M{self.pin} -> Home | Speed: {speed}", override=True)
        
        # move motor to home position and check for stalls
        while True:
            current_time = time.time()
            # initial count position has not changed -> already home or jammed
            if not override_initial_timeout:
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
                timed_out = True
                break
            
            await asyncio.sleep(0.01) # yield control back to event

        self.stop()
        return self.counts, timed_out
        
    async def to(self, target: float, speed: float = constants.mid_speed, timeout: int  = constants.to_position_timeout) -> tuple[int, bool, int]:
        """Move the motor to a specific position relative to `max_counts` at a specific speed"""

        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        target_counts = int((target / 1 ) * (constants.max_counts))
        timed_out = False 
        
        log.info(f'Moving: M{self.pin} ({self.counts} -> {target_counts}) at speed {speed}')

        # change direction based on target position
        if target_counts > self.counts:
            self.direction = constants.down
        else:
            self.direction = constants.up

        start_time = time.time() # track time

        if self.counts == target_counts:
            log.success(f"Motor {self.pin} already at target position")
            return self.counts, timed_out, seconds_elapsed(start_time)
        
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        self.set(speed, self.direction) # set the motor in the correct direction
        
        while True:
            # check if the motor has reached the target position
            if self.counts == target_counts:
                log.success(f"Motor {self.pin} has reached target position")
                break
            # motor has timed out -> stop the motor
            if time.time() - start_time > timeout:
                self._error(f"Motor {self.pin} timed out moving to target position, disabling...")
                timed_out = True
                break

            await asyncio.sleep(0.01) # yield control back to event
        
        self.encoder_feedback_disabled = True # stop incrementing encoder counts
        self.stop()
        return self.counts, timed_out, seconds_elapsed(start_time)
    
    # -------------------------------- CALIBRATION ------------------------------- #
    async def _find_cps(self):
        """Find counts per second of the motor in both directions"""
        log.info(f"Finding cps up and down for M{self.pin}", override=True)

         # ------------------------------- find cps down ------------------------------ #
        if self.cps_down is None:
            log.info(f"Calculating M{self.pin} cps down", override=True)
            self.direction = constants.down 
            self.set(constants.calibration_speed, self.direction) # set the motor to the calibration speed

            start = time.time()
            # move the motor to the calibration position
            while self.counts != constants.calibration_counts:
                # motor has timed out -> error
                if time.time() - start > constants.calibration_timeout:
                    self._error(f"Motor {self.pin} timed out calibrating, disabling...")
                    return
                
                await asyncio.sleep(0.01) # yield control back to event
                
            self.stop()
            
            down_time = time.time() - start # time taken to move to calibration position
            self.cps_down = constants.calibration_counts / down_time # time per count

        # -------------------------------- find cps up ------------------------------- #
        if self.cps_up is None:
            log.info(f"Calculating M{self.pin} up cps", override=True)

            start = time.time()
            await self.to_home(speed=constants.calibration_speed)
            
            up_time = time.time() - start
            self.cps_up = constants.calibration_counts / up_time

    async def _find_mins(self):
        """Find the minimum speed of the motor in both directions"""
        log.info(f"Finding min speeds for M{self.pin}")
        
        # ---------------------------- find min down speed --------------------------- #
        log.info(f"Finding min down speed for M{self.pin}", override=True)

        # move the motor to the calibration position at different speeds and look for timeout (down)
        step = 0.01
        current_throttle = 0.4
        neutral_down = None

        while True:
            current_throttle -= step
            log.info(f"Testing speed: {step} (down)", override=True)
            _, timed_out, _ = await self.to(0.1, current_throttle, constants.calibration_to_position_timeout)

            if not timed_out:
                neutral_down = current_throttle - step
                break

        log.info(f"Neutral down: {neutral_down}")


        # for current_speed in reversed([round(x * constants.calibration_speed_step, 2) for x in range(0, constants.calibration_total_steps)]):
        #     log.info(f"Testing speed: {current_speed}")
        #     _, timed_out, _  = await self.to(0.2, current_speed)

        #     # motor has timed out -> found slowest speed
        #     if timed_out:
        #         break

        #     self.min_down_speed = current_speed
        #     await self.to_home() # return home for next iteration

        # # ---------------------------- find min up speed --------------------------- #
        # log.info(f"Finding min up speed for M{self.pin}", override=True)
        # self.direction = constants.up

        # # move to calibration position -> try to move home at different speeds and look for timeout (up)
        # for current_speed in reversed([round(x * constants.calibration_speed_step, 2) for x in range(0, constants.calibration_total_steps)]):
        #     log.info(f"Testing speed: {current_speed}", override=True)
        #     await self.to(0.2) # move to calibration position
        #     _, timed_out = await self.to_home(current_speed, override_initial_timeout=True) # move home and check for timeouts

        #     # motor has timed out -> found slowest speed
        #     if timed_out:
        #         break

        #     self.min_up_speed = current_speed

        log.success(f"M{self.pin} | min down speed: {self.min_down_speed} | min up speed: {self.min_up_speed}")

        # cleanup
        # await self.to_home()
        self.encoder_feedback_disabled = True
        step = 0.01
        current_throttle = 0
        neutral_down = None

        while True:
            current_throttle += step
            log.info(f"Testing speed: {step} (down)")
            _, timed_out, _ = await self.to(0.1, current_throttle, constants.calibration_to_position_timeout)

            if not timed_out:
                neutral_down = current_throttle - step
                break

        print(f"Neutral down: {neutral_down}")

    async def calibrate(self, data: list[Optional[float]] = [None, None, None, None, None]):
        """Calibrate the motor to determine lower and upper bounds of motor speed"""

        # load calibration data if available
        if data[DataMode.cps_down.value] is not None:
            log.info(f"M{self.pin} | cps down already calibrated")
            self.cps_down = data[DataMode.cps_down.value]
        if data[DataMode.cps_up.value] is not None:
            log.info(f"M{self.pin} | cps up already calibrated")
            self.cps_up = data[DataMode.cps_up.value]
        if data[DataMode.min_down_speed.value] is not None:
            log.info(f"M{self.pin} | min down speed already calibrated")
            self.min_down_speed = data[DataMode.min_down_speed.value]
        if data[DataMode.min_up_speed.value] is not None:
            log.info(f"M{self.pin} | min up speed already calibrated")
            self.min_up_speed = data[DataMode.min_up_speed.value]

        # ensure motor is at home before calibrating
        # if not self.is_home():
        #     await self.to_home()

        self.encoder_feedback_disabled = False # start incrementing encoder counts

        log.info(f"Calibrating M{self.pin}")

        # find up and down cps if either is not present
        # if self.cps_down is None or self.cps_up is None:
        #     await self._find_cps()
        
        # find min speeds if either is not present
        if self.min_down_speed is None or self.min_up_speed is None:
            await self._find_mins()
        
        self.encoder_feedback_disabled = True

    def is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0
    
    def __str__(self) -> str:
        return f"Motor {self.pin} | counts: {self.counts} | cps up: {self.cps_up} | cps down: {self.cps_down} | up boost: {self.up_boost} | down boost: {self.down_boost}"