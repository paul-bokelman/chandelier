from typing import Optional
from store import SingularCalibrationData
import time
import asyncio
import RPi.GPIO as GPIO
from adafruit_servokit import ContinuousServo
from lib.utils import log
from configuration.config import config

# todo: all disabled functionality should be present here (don't run motor if disabled)

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, channel: int, servo: ContinuousServo) -> None:
        self.channel = channel
        self.last_read_time = None
        self.direction = config.get('down') # direction of motor (used in encoder callback)
        self.encoder_pin = config.get('encoder_pins')[self.channel]
        self.servo = servo
        self.servo.set_pulse_width_range(1000, 2000) 
        self.disabled = False

        # todo: counts should be none unless calibrated (position unknown)
        self.counts = -1  # current count position, -1 indicates that the motor has not been calibrated

        # calibration data
        self.lower_neutral = None
        self.upper_neutral = None
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.throttle_down = None # relative throttle for moving down at slow preset
        self.throttle_up = None # relative throttle for moving up at slow preset

        # detect when encoder is triggered (matches 0's)
        # todo: use pigpio for better performance (https://chatgpt.com/share/67042694-7590-8009-ba97-8d07e8241e36)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

        if self.channel in config.get('initial_disabled_motors'):
            self.disabled = True
            log.warning(f"M{self.channel} | Motor disabled")

    def _encoder_callback(self, _):
        """Callback function for encoder"""
        self.counts += self.direction * 1 # increment or decrement counts based on direction
        self.last_read_time = time.time() # track last read time

        # log counts and direction
        if not config.get('suppress_count_logging'):
            log.info(f"M{self.channel} | count: {self.counts} | direction: {'down' if self.direction == config.get('down') else 'up'}")

    def _set_home_state(self):
        """Set motor to home state"""
        self.last_read_time = None
        self.direction = config.get('down')
        self.counts = 0

    def _clm(self, function_name, **kwargs):
        """Construct consistent log message for motor functions"""
        message = f"M{self.channel} | {function_name}"
        for key, value in kwargs.items():
            message += f" | {key}: {value}"
        return message
    
    # todo: may need work related to motor recovery
    def _disable(self, reason: str = "Unknown"):
        log.error(self._clm("Disable", message="Disabling motor", reason=reason))
        self.disabled = True

    def _notify_disabled(self, parent_function: str):
        """Flag motor as disabled"""
        log.warning(self._clm(parent_function, message="Motor is disabled"))

    def _is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0

    def set(self, throttle: Optional[float] = None , direction: Optional[int] = None):
        """Set the motor throttle and direction, no direction -> use offset from neutral (requires direction)"""

        # specific value -> set throttle to that value and ignore direction
        if throttle is not None:
            if not (-1 <= throttle <= 1):
                raise ValueError("Throttle must be between -1 and 1")
            self.servo.throttle = throttle
            return
        
        # check for direction
        if direction is None:
            raise ValueError("Direction must be set when using offset throttle")
        
        # check for valid direction
        if direction not in [config.get('up'), config.get('down')]:
            raise ValueError("Direction must be up or down for preset throttle")
        
        # neutral positions must be calibrated and set for present throttles
        if self.lower_neutral is None and self.upper_neutral is None:
            raise ValueError("Neutral positions not found, can't set offset throttle")
        
        # set throttle based on direction
        neutral = self.lower_neutral if direction == config.get('up') else self.upper_neutral
        self.servo.throttle = neutral + (direction * config.get('throttle_offset'))

    def stop(self):
        """Stop the motor"""
        log.info(self._clm("Stop"))
        self.servo._pwm_out.duty_cycle = 0  

    async def to_home(self, throttle: Optional[float] = None) -> tuple[bool, float, bool]:
        """
        Move the motor to the home position

        Parameters:
            throttle (float, optional): throttle to move at, defaults to None

        Returns:
            tuple(bool, float, bool): timed_out, time_elapsed, aborted
        """
        log.info(self._clm("To Home", message="Moving Home"))

        # already home -> return early
        if self._is_home(): 
            log.success(self._clm("To Home", message="Motor already at home"))
            return False, 0.0, False

        # move to 0 count position with default throttle
        log.info(self._clm("To Home", message="Moving Home"))
        return await self.to(0.0, throttle, config.get('to_home_timeout'))

    async def move(
        self, 
        n_counts: int,
        throttle: Optional[float] = None,
        direction: Optional[int] = None,
        timeout: int = config.get('to_position_timeout'),
    ) -> tuple[bool, float, bool]:
        """
        Move the motor a specific number of counts at a specific throttle
        
        Parameters:
            n_counts (int): number of counts to move
            throttle (float, optional): throttle to move at, defaults to None
            direction (int, optional): direction to move in, defaults to None
            timeout (int, optional): max time to move, defaults to config.get('to_position_timeout')

        Returns:
            tuple(bool, float, bool): timed_out, time_elapsed, aborted
        """

        # motor is disabled -> abort
        if self.disabled:
            self._notify_disabled("Move")
            return False, 0.0, True

        max_counts: int = config.get('max_counts')
        
        # ensure n_counts is within bounds
        if n_counts > max_counts:
            raise ValueError("Counts must be less than max counts")
        if n_counts < 0:
            raise ValueError("Counts must be greater than 0")

        log.info(self._clm("Move", start_counts=self.counts, n_counts=n_counts, direction=self.direction, throttle=throttle))

        start_time = time.time() # track time
        start_counts = self.counts # track start position
        timed_out = False

        self.set(throttle, direction) # start motor

        while True:
            # check if the motor has reached the target position
            if abs(self.counts - start_counts) == n_counts:
                log.success(self._clm("Move", message="Motor has reached target position"))
                break
            
            # hasn't reached target position before timeout -> exit
            if time.time() - start_time > timeout:
                log.error(self._clm("Move", message="Motor timed out"))
                timed_out = True
                break

            await asyncio.sleep(0.01) # yield control back to event

        time_elapsed = time.time() - start_time

        self.stop()
        return timed_out, time_elapsed, False
    
    async def to(
            self, 
            target: float, 
            throttle: Optional[float] = None,
            timeout: int = config.get('to_position_timeout'),
    ) -> tuple[bool, float, bool]:
        """
        Move the motor to a specific position scaled between 0 and 1
        
        Parameters:
            target (float): target position to move to, between 0 and 1
            throttle (float, optional): throttle to move at, defaults to None
            direction (int, optional): direction to move in, defaults to None
            timeout (int, optional): max time to move, defaults to config.get('to_position_timeout')

        Returns:
            tuple(bool, float, bool): timed_out, time_elapsed, aborted
        """
        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        max_counts: int = config.get('max_counts')
        target_counts = int(target * max_counts)
        n_counts = abs(target_counts - self.counts)
        self.direction = config.get('up') if n_counts < 0 else config.get('down')

        log.info(self._clm("To", message=f"({self.counts} -> {target_counts})", throttle=throttle))
        return await self.move(n_counts, throttle, self.direction, timeout) # move to target position
    
    # -------------------------------- CALIBRATION ------------------------------- #
    async def calibrate_relative_throttles(self, target_up_cps: float, target_down_cps: float):
        """
        Find and assign relative throttles based on global cps data (only configured for slow speed)
        
        Parameters:
            target_up_cps (float): target cps moving up
            target_down_cps (float): target cps moving
        """
        log.info(self._clm("CRT", message="Calibrating relative throttles"))
        
        # pre-checks
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")
        if self.cps_down is None or self.cps_up is None:
            raise ValueError("CPS not found")

        if self.throttle_down is not None and self.throttle_up is not None:
            log.info(self._clm("CRT", message="Throttles already calibrated"))
            return

        # move back home if not already 
        if not self._is_home():
            await self.to_home()

        await self.to(0.1) # move to buffer position (avoid hitting top on up)

        # descent configuration
        error_margin = 0.03
        step_size = 0.01
        
        # scaling factors
        up_factor: float = 15
        down_factor: float = 15

        # persistent variables
        previous_down_cps = self.cps_down
        previous_up_cps = self.cps_up
        down_throttle = self.upper_neutral + (step_size * down_factor)
        up_throttle = self.lower_neutral - (step_size * up_factor)
        found_down_throttle = False
        found_up_throttle = False

        log.info(self._clm("CRT", target_down_cps=target_down_cps, target_up_cps=target_up_cps))

        def measure_and_adjust_throttle(
                direction: int, 
                time_elapsed: float, 
                throttle: float, 
                previous_cps: float, 
                factor: float
        ) -> tuple[float, float, float, bool]:
            """
            Measure cps and adjust throttle based on error for a specific direction

            Parameters:
                direction (int): direction to move in
                time_elapsed (float): time taken to move
                throttle (float): current throttle
                previous_cps (float): previous cps
                factor (float): scaling factor

            Returns:
                tuple(float, float, float, bool): cps, new_throttle, new_factor, found_throttle
            
            """
            is_down = direction == config.get('down')
            target_cps = target_down_cps if is_down else target_up_cps
            found_throttle = False
            cps: float = config.get('calibration_counts') / time_elapsed # calculate cps
            error = target_cps - cps # calculate error

            log.info(self._clm("CRT", message="Measuring and adjusting throttle", cps=cps, error=error))

            # target in between previous and current cps -> reduce scale factor
            if previous_cps < target_cps < cps or previous_cps > target_cps > cps:
                log.info(self._clm("CRT", message=f"Reducing {'down' if is_down else 'up'} factor"))
                new_factor = factor * 0.75 #/ should be proportional to error

            # adjust throttle based on error
            direction = 1 if cps < target_cps else -1 # determine direction
            new_throttle = throttle + (direction * new_factor * step_size) # adjust throttle
            log.info(self._clm("CRT", message=f"{'Reducing' if direction == -1 else 'Increasing'} {'down' if is_down else 'up'} throttle", new_throttle=new_throttle))

            # throttle is within error margin -> found throttle
            if abs(error) <= error_margin:
                log.success(self._clm("CRT", message="Throttle found", throttle=new_throttle))
                found_throttle = True

            return cps, new_throttle, new_factor, found_throttle

        # move to calibration position and measure cps until within error
        while not found_down_throttle or not found_up_throttle:
            # move down into down position
            down_timed_out, down_time_elapsed, aborted = await self.move(n_counts=config.get('calibration_counts'), throttle=down_throttle, timeout=config.get('calibration_timeout'))
            
            # move timed out -> exit
            if down_timed_out:
                log.error(self._clm("CRT", message="Motor timed out moving down", throttle=down_throttle, time_elapsed=down_time_elapsed))
                raise ValueError("Motor timed out moving down")
        
            # move back to previous position to measure up cps
            up_timed_out, up_time_elapsed, aborted = await self.move(n_counts=config.get('calibration_counts'), throttle=up_throttle, direction=config.get('up'), timeout=config.get('calibration_timeout'))

            # move timed out -> exit
            if up_timed_out:
                log.error(self._clm("CRT", message="Motor timed out moving up", throttle=up_throttle, time_elapsed=up_time_elapsed))
                raise ValueError("Motor timed out moving up")

            # not found down throttle -> measure and adjust
            if not found_down_throttle:
                previous_down_cps, down_throttle, down_factor, found_down_throttle = measure_and_adjust_throttle(direction=config.get('down'), time_elapsed=down_time_elapsed, throttle=down_throttle, previous_cps=previous_down_cps, factor=down_factor)

            # not found up throttle -> measure and adjust
            if not found_up_throttle:
                previous_up_cps, up_throttle, up_factor, found_up_throttle = measure_and_adjust_throttle(direction=config.get('up'), time_elapsed=up_time_elapsed, throttle=up_throttle, previous_cps=previous_up_cps, factor=up_factor)

        self.throttle_down = down_throttle
        self.throttle_up = up_throttle
        log.success(self._clm("CRT", throttle_down=self.throttle_down, throttle_up=self.throttle_up))

    async def _find_cps(self):
        """Find counts per second of the motor in both directions"""
        log.info(self._clm("Find CPS", message="Finding cps up and down"))

        # neutral positions must be calibrated and set for present throttles
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")

        # move back home if not already
        if not self._is_home():
            await self.to_home(config.get('uncalibrated_throttle'))

        await self.to(0.1) # move to buffer position 

         # ------------------------------- find cps down ------------------------------ #
        if self.cps_down is None:
            log.info(self._clm("Find CPS", message="Finding cps down"))

            # move to down to calibration position and measure time
            timed_out, time_elapsed, aborted = await self.move(
                n_counts=config.get('calibration_counts'),
                direction=config.get('down'),
                timeout=config.get('calibration_timeout')
            ) 

            if timed_out:
                log.error(self._clm("Find CPS", message="Motor timed out finding cps down"))
                raise ValueError("Motor timed out finding cps down")

            self.cps_down = config.get('calibration_counts') / time_elapsed # compute cps down
            log.info(self._clm("Find CPS", cps_down=self.cps_down))

        # -------------------------------- find cps up ------------------------------- #
        if self.cps_up is None:
            log.info(self._clm("Find CPS", message="Finding cps up"))
            
            # move to up to calibration position and measure time
            timed_out, time_elapsed, aborted = await self.move(
                n_counts=config.get('calibration_counts'),
                direction=config.get('up'),
                timeout=config.get('calibration_timeout')
            )

            if timed_out:
                log.error(self._clm("Find CPS", message="Motor timed out finding cps up"))
                raise ValueError("Motor timed out finding cps up")
            
            self.cps_up = config.get('calibration_counts') / time_elapsed # compute cps up
            log.info(self._clm("Find CPS", cps_up=self.cps_up))

        log.success(self._clm("Find CPS", cps_down=self.cps_down, cps_up=self.cps_up))

    async def _find_neutrals(self):
        """Find the lower and upper neutral positions of motor"""
        log.info(self._clm("Find Neutrals", message="Finding neutral positions"))

        # ensure motor is at home position
        if not self._is_home():
            await self.to_home(config.get('uncalibrated_throttle'))

        step = 0.01 
        current_throttle = 0.35
        
        # continually decrease throttle until both neutral positions are found
        while self.upper_neutral is None or self.lower_neutral is None:
            current_throttle = round(current_throttle - step, 2)
            log.info(self._clm("Find Neutrals", current_throttle=current_throttle))
            timed_out, time_elapsed, aborted = await self.move(n_counts=2, throttle=current_throttle, timeout=config.get("calibration_timeout"))

            # initial throttle has timed out -> found upper neutral
            if self.upper_neutral is None and timed_out:
                log.info(self._clm("Find Neutrals", message=f"Upper neutral found: {current_throttle}"))
                self.upper_neutral = current_throttle

            # upper neutral found and motor has not timed out -> found lower neutral
            if self.lower_neutral is None and not timed_out and self.upper_neutral is not None:
                log.info(self._clm("Find Neutrals", message=f"Lower neutral found: {current_throttle}"))
                self.lower_neutral = current_throttle + step # add step to account for last iteration

        # ensure both neutral positions were found
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions calibrated incorrectly")

        log.info(self._clm("Find Neutrals", lower_neutral=self.lower_neutral, upper_neutral=self.upper_neutral))

    async def calibrate_independent(self):
        """Calibrate motors independent variables by finding neutral positions and cps in both directions"""
        log.info(self._clm("Calibrate Independent", message="Calibrating Motor"))

        # find neutrals if either is not present
        if self.lower_neutral is None or self.upper_neutral is None:
            await self._find_neutrals()

        # find cps if either is not present
        if self.cps_down is None or self.cps_up is None:
            await self._find_cps()

        await self.to_home() # move back to home position for next calibration
        
    def get_calibration_data(self) -> SingularCalibrationData:
        """
        Get calibration data for this motor

        Returns:
            SingularCalibrationData: calibration data for this motor
        """
        return {
            "cps_down": self.cps_down,
            "cps_up": self.cps_up,
            "lower_neutral": self.lower_neutral,
            "upper_neutral": self.upper_neutral,
            "throttle_down": self.throttle_down,
            "throttle_up": self.throttle_up
        }