from typing import Optional, Union
from adafruit_servokit import ContinuousServo
import time
import asyncio
import RPi.GPIO as GPIO
from lib.store import DataMode
from lib.utils import log, to_pulse, seconds_elapsed
import constants

Throttle = Union[float, constants.ThrottlePresets]

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, channel: int, servo: ContinuousServo) -> None:
        self.channel = channel
        self.last_read_time = None
        self.encoder_feedback_disabled = False # get feedback from encoder
        self.direction = constants.down # direction of motor
        self.encoder_pin = constants.encoder_pins[self.channel]
        self.servo = servo
        self.servo.set_pulse_width_range(1000, 2000) 

        # calibrated data
        # todo: counts should be none unless calibrated (position unknown)
        self.counts = -1  # current count position, -1 indicates that the motor has not been calibrated
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.up_boost: Optional[float] = None # percentage boost needed relative to others, measured at (calibrated counts @ calibrated speed)
        self.down_boost: Optional[float] = None # percentage boost needed relative to others, measured at (calibrated counts @ calibrated speed)

        self.lower_neutral = None
        self.upper_neutral = None

        self.min_throttle_down = None
        self.min_throttle_up = None

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

    def _encoder_callback(self, channel: int):
        """Callback function for encoder"""
        if self.encoder_feedback_disabled:
            return

        self.counts += self.direction * 1 # increment encoder count
        self.last_read_time = time.time()
        log.info(f"M{self.channel} | count: {self.counts} | direction: {'down' if self.direction == constants.down else 'up'}")

    def _at_home(self):
        """Set motor home state"""
        self.encoder_feedback_disabled = True
        self.last_read_time = None
        self.direction = constants.down
        self.counts = 0

    def _clm(self, function_name, **kwargs):
        """Construct consistent log message for motor functions"""
        message = f"M{self.channel} | {function_name}"
        for key, value in kwargs.items():
            message += f" | {key}: {value}"
        return message

    def set(self, throttle: Throttle = constants.ThrottlePresets.SLOW, direction: Optional[int] = None):
        """Set a specific servo to a specific or set throttle and direction"""
        assert isinstance(throttle, (constants.ThrottlePresets, float)), f"Throttle must be a float or ThrottlePresets, got {type(throttle)}"

        # specific value -> set throttle
        if isinstance(throttle, float):
            assert direction is None, "Direction not available for specific throttle"
            assert -1 <= throttle <= 1, "Throttle must be between -1 and 1"
            self.servo.throttle = throttle
            return
        
        # neutral positions must be calibrated and set for present throttles
        assert self.lower_neutral is not None and self.upper_neutral is not None, "Neutral positions not found"
        assert direction in [constants.up, constants.down], "Direction must be up or down for preset throttle"
        
        # set throttle based on preset (relative to neutral)
        offset = throttle.value
        if direction == constants.up:
            self.servo.throttle = self.lower_neutral - offset
        else:
            self.servo.throttle = self.upper_neutral + offset


    def stop(self):
        """Stop the motor"""
        log.info(self._clm("Stop"), override=True)
        self.servo._pwm_out.duty_cycle = 0  

    async def to_home(self, throttle: Throttle = constants.ThrottlePresets.SLOW, override_initial_timeout = False) -> tuple[int, bool]:
        """Move the motor to the home position (0 count)"""
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        timed_out = False
        log.info(self._clm("To Home", message="Moving Home"), override=True)

        if self.is_home(): 
            log.success(self._clm("To Home", message="Motor already at home"))
            return self.counts, timed_out

        self.set(throttle=throttle, direction=constants.up if isinstance(throttle, constants.ThrottlePresets) else None)
        self.last_read_time = None 
        start_time = time.time()

        log.info(self._clm("To Home", message="Moving Home"), override=True)
        
        # move motor to home position and check for stalls
        while True:
            current_time = time.time()
            # initial count position has not changed -> already home or jammed
            if not override_initial_timeout:
                if current_time - start_time > constants.to_home_initial_timeout and self.last_read_time is None:
                    log.success(self._clm("To Home", message="Motor at home position"))
                    self._at_home()
                    break
            # check if the motor has reached home
            if self.last_read_time is not None and current_time - self.last_read_time > constants.to_home_max_interval:
                log.success(self._clm("To Home", message="Motor at home position", time=seconds_elapsed(start_time)))
                self._at_home()
                break
            # if the motor has timed out, stop the motor
            if current_time - start_time > constants.to_home_timeout:
                log.error(self._clm("To Home", message="Motor timed out", timeout=constants.to_home_timeout))
                timed_out = True
                break
            
            await asyncio.sleep(0.01) # yield control back to event

        self.stop()
        return self.counts, timed_out
    
    async def move(
        self, 
        n_counts: int,
        throttle: Throttle = constants.ThrottlePresets.SLOW, 
        direction: Optional[int] = None,
        timeout: int = constants.to_position_timeout
    ) -> tuple[int, bool, int]:
        """Move the motor n number of counts at a specific speed"""
        timed_out = False

        log.info(self._clm("Move", message=f"({self.counts} -> {self.counts + n_counts}), Throttle: {throttle.name if isinstance(throttle, constants.ThrottlePresets) else throttle}"))
        
        if self.counts + n_counts < 0 or self.counts + n_counts > constants.max_counts:
            raise ValueError("Counts must be between 0 and max counts")
        
        if n_counts == 0:
            log.success(self._clm("Move", message="No counts to move"))
            return self.counts, timed_out, 0
        
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        self.set(throttle, direction)

        start_time = time.time() # track time
        start_counts = self.counts
        
        while True:
            # check if the motor has reached the target position (have to check for abs because direction may be unknown)
            if abs(self.counts - start_counts) == n_counts:
                log.success(self._clm("Move", message="Motor has reached target position"))
                break
            # hasn't reached target position before timeout -> exit
            if time.time() - start_time > timeout:
                log.error(self._clm("Move", message="Motor timed out", timeout=timeout))
                timed_out = True
                break
            await asyncio.sleep(0.01) # yield control back to event

        self.encoder_feedback_disabled = True # stop incrementing encoder counts
        self.stop()
        return self.counts, timed_out, seconds_elapsed(start_time)
        
    async def to(
            self, target: float, 
            throttle_offset: constants.ThrottlePresets = constants.ThrottlePresets.SLOW, 
            timeout: int = constants.to_position_timeout
    ) -> tuple[int, bool, int]:
        """Move the motor to a specific position relative to `max_counts` at a specific speed"""

        log.info(self._clm("To", message=f"Target: {target}, Throttle: {throttle_offset.name}"))

        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        target_counts = int((target / 1) * (constants.max_counts))
        n_counts = target_counts - self.counts

        self.direction = constants.down if target_counts > self.counts else constants.up
        return await self.move(n_counts, throttle_offset, self.direction, timeout) # move to target position
    
    # -------------------------------- CALIBRATION ------------------------------- #

    async def _find_cps(self):
        """Find cps of motor"""
        pass

    async def _find_relative_throttles(self):
        """Find relative throttles based on target cps from throttle presets"""
        log.info(self._clm("Find CPS", message="Finding cps up and down"), override=True)

        # neutral positions must be calibrated and set for present throttles
        assert self.lower_neutral is not None and self.upper_neutral is not None, "Neutral positions not found"

        # move back home if not already (allows _find_cps to be called separately from _find_neutrals)
        if not self.is_home():
            await self.to_home()

        target_cps = 1 # slow throttle preset
        error = 0.01 # error margin
        step = 0.01 # step size
        step_multiplier = 1 # step size multiplier (scales based on distance)

        # gradient descent to find cps up and down for slow throttle
        log.info(self._clm("Find CPS", message="Finding throttle for target CPS"), override=True)

        previous_cps = None
        current_throttle = self.upper_neutral + step * step_multiplier # move down

        # move to calibration position and measure cps until within error
        while previous_cps is None or abs(target_cps - previous_cps) > error:
            start = time.time()
            await self.move(n_counts=constants.calibration_counts, throttle=current_throttle, timeout=constants.calibration_timeout)
            current_cps = constants.calibration_counts / (time.time() - start)
            previous_cps = current_cps

            # throttle too low -> increase throttle
            if current_cps < target_cps:
                current_throttle += step * step_multiplier
                log.info(self._clm("Find CPS", message="Increasing throttle", throttle=current_throttle, cps=current_cps), override=True)

            # throttle too high -> decrease throttle and decrease step size
            else:
                current_throttle -= step * step_multiplier
                step_multiplier /= 2
                log.info(self._clm("Find CPS", message="Decreasing throttle", throttle=current_throttle, cps=current_cps), override=True)

            #? could calculate cps up here
            await self.to_home() # move back to home position for next iteration

        log.info(self._clm("Find CPS", message="Slow throttle found", relative_throttle=current_throttle), override=True)


        #  # ------------------------------- find cps down ------------------------------ #
        # if self.cps_down is None:
        #     log.info(self._clm("Find CPS", message="Finding cps down"), override=True)
        #     start = time.time()

        #     # move to calibration position
        #     await self.move(
        #         n_counts=constants.calibration_counts, 
        #         throttle=constants.ThrottlePresets.SLOW, 
        #         direction=constants.down, 
        #         timeout=constants.calibration_timeout
        #     ) 

        #     self.cps_down = constants.calibration_counts / (time.time() - start) # compute cps down
        #     log.info(self._clm("Find CPS", cps_down=self.cps_down), override=True)

        # # -------------------------------- find cps up ------------------------------- #
        # if self.cps_up is None:
        #     log.info(self._clm("Find CPS", message="Finding cps up"), override=True)
        #     start = time.time()

        #     # move to home position at slow speed
        #     await self.to_home(throttle=constants.ThrottlePresets.SLOW)
            
        #     self.cps_up = constants.calibration_counts / (time.time() - start) # compute cps up
        #     log.info(self._clm("Find CPS", cps_up=self.cps_up), override=True)

        log.success(self._clm("Find CPS", cps_down=self.cps_down, cps_up=self.cps_up))

    async def _find_neutrals(self):
        """Find the lower and upper neutral positions of servo motor"""
        log.info(self._clm("Find Neutrals", message="Finding neutral positions"), override=True)

        # ensure motor is at home position
        if not self.is_home():
            await self.to_home(constants.uncalibrated_home_throttle)

        step = 0.01 #? should be in constants
        current_throttle = 0.35 #? should be in constants
        
        # continually decrease throttle until both neutral positions are found
        while self.upper_neutral is None or self.lower_neutral is None:
            current_throttle = round(current_throttle - step, 2)
            log.info(self._clm("Find Neutrals", current_throttle=current_throttle), override=True)
            _, timed_out, _ = await self.move(n_counts=2, throttle=current_throttle, timeout=constants.calibration_to_position_timeout)

            # initial throttle has timed out -> found upper neutral
            if self.upper_neutral is None and timed_out:
                log.info(self._clm("Find Neutrals", message=f"Upper neutral found: {current_throttle}"), override=True)
                self.upper_neutral = current_throttle

            # upper neutral found and motor has not timed out -> found lower neutral
            if self.lower_neutral is None and not timed_out and self.upper_neutral is not None:
                log.info(self._clm("Find Neutrals", message=f"Lower neutral found: {current_throttle}"), override=True)
                self.lower_neutral = current_throttle + step # add step to account for last iteration

        log.info(self._clm("Find Neutrals", lower_neutral=self.lower_neutral, upper_neutral=self.upper_neutral), override=True)

    async def calibrate(self, data: list[Optional[float]] = [None, None, None, None]):
        """Calibrate the motor to determine lower and upper bounds of motor speed"""
        log.info(self._clm("Calibrate", message="Calibrating Motor"))

        # load calibration data if available
        if data[DataMode.CPS_DOWN.value] is not None:
            log.info(self._clm("Calibrate", message="cps down already calibrated"), override=True)
            self.cps_down = data[DataMode.CPS_DOWN.value]
        if data[DataMode.CPS_UP.value] is not None:
            log.info(self._clm("Calibrate", message="cps up already calibrated"), override=True)
            self.cps_up = data[DataMode.CPS_UP.value]
        if data[DataMode.LOWER_NEUTRAL.value] is not None:
            log.info(self._clm("Calibrate", message="lower neutral already calibrated"), override=True)
            self.lower_neutral = data[DataMode.LOWER_NEUTRAL.value]
        if data[DataMode.UPPER_NEUTRAL.value] is not None:
            log.info(self._clm("Calibrate", message="upper neutral already calibrated"), override=True)
            self.upper_neutral = data[DataMode.UPPER_NEUTRAL.value]

        self.encoder_feedback_disabled = False # start incrementing encoder counts

        # find neutrals if either is not present
        if self.lower_neutral is None or self.upper_neutral is None:
            await self._find_neutrals()

        # find relative throttles if either is not present
        if self.cps_down is None or self.cps_up is None:
            await self._find_relative_throttles()

        await self.to_home() # move back with calibrated neutral positions
        
        self.encoder_feedback_disabled = True

        # ensure calibration was successful
        # assert self.cps_down is not None and self.cps_up is not None, "CPS incorrectly calibrated"
        assert self.lower_neutral is not None and self.upper_neutral is not None, "Min speeds incorrectly calibrated"
        assert self.lower_neutral is not None and self.upper_neutral is not None, "Neutral positions not found"

    def is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0
    
    def __str__(self) -> str:
        return f"Motor {self.channel} | counts: {self.counts} | cps up: {self.cps_up} | cps down: {self.cps_down} | up boost: {self.up_boost} | down boost: {self.down_boost}"