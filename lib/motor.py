from typing import Optional, Union, cast
from enum import Enum
import time
import asyncio
from adafruit_servokit import ContinuousServo
from lib.utils import log
from configuration.config import config

try:
    import RPi.GPIO as GPIO # type: ignore
except ImportError:
    import Mock.GPIO as GPIO

class MoveException(Enum):
    """Move error types"""
    TIMED_OUT = "Timed out"
    STALLED = "Stalled"

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, channel: int, servo: ContinuousServo) -> None:
        # general data
        self.channel = channel
        self.direction: int = config.get('down') # direction of motor (used in encoder callback)
        self.servo = servo
        self.servo.set_pulse_width_range(1000, 2000) 

        # state data
        self.disabled = False # if the motor is disabled
        self.dead = False # if the motor is dead
        self.recover_attempts = 0 # number of times the motor has attempted to recover

        # encoder data
        self.encoder_pin = config.get('encoder_pins')[self.channel]
        self.counts = -1  # current count position, -1 indicates that the motor position is unknown
        self.measuring_cps = False # if the encoder is currently being read
        self.last_read_time: Union[float, None] = time.time() # previous time the encoder was read
        self.current_cps: float = 0 # current counts per second

        # calibration data
        self.lower_neutral = None
        self.upper_neutral = None
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.throttle_down = None # relative throttle for moving down at slow preset
        self.throttle_up = None # relative throttle for moving up at slow preset

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

        # mark motor as dead if it is in the initial disabled list
        if self.channel in config.get('initial_disabled_motors'):
            self.disabled = True
            self.dead = True
            log.warning(f"M{self.channel} | Motor dead")

    def _encoder_callback(self, _):
        """Callback function for encoder"""
        self.counts += self.direction * 1 # increment or decrement counts based on direction

        if self.measuring_cps:
            current_time = time.time() # get current time
            time_elapsed = current_time - self.previous_read # calculate time elapsed
            self.current_cps = 1 / time_elapsed # calculate counts per second
            self.previous_read = current_time # update previous read time

        # log counts and direction
        if not config.get('suppress_count_logging'):
            log.info(f"M{self.channel} | count: {self.counts} | direction: {'down' if self.direction == config.get('down') else 'up'} | cps: {self.current_cps if self.measuring_cps else 'N/A'}")

    def _reset_cps_readings(self):
        """Reset cps readings (Doesn't stop measuring)"""
        self.current_cps = 0
        self.last_read_time = None

    def _start_measuring_cps(self):
        """Start measuring cps"""
        self._reset_cps_readings()
        self.measuring_cps = True

    def _stop_measuring_cps(self):
        """Stop measuring cps"""
        self.measuring_cps = False

    def _set_home_state(self):
        """Set motor to home state"""
        self.direction = config.get('down')
        self.counts = 0

    def _clm(self, function_name, **kwargs):
        """
        Construct consistent log message for motor functions

        Parameters:
            function_name (str): name of the function
            **kwargs: additional keyword arguments to include in the log message
        """
        message = f"M{self.channel} | {function_name}"
        for key, value in kwargs.items():
            message += f" | {key}: {value}"
        return message
    
    def _disable(self, reason: str = "Unknown"):
        """
        Disable the motor
        
        Parameters:
            reason (str, optional): reason for disabling, defaults to "Unknown"
        """
        log.error(self._clm("Disable", message="Disabling motor", reason=reason))
        self.disabled = True

    def _is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0
    
    @staticmethod
    def _handle_disabled(f):
        """Decorator to check and properly handle disabled state"""
        def wrapper(self: 'Motor', *args, **kwargs):
            
            # if the motor is disabled -> notify and abort
            if self.disabled:
                log.warning(self._clm("Handle Disabled", message="Motor is disabled"))
                return asyncio.sleep(0) #/ hacky way to make asyncio happy
            
            # todo: revisit this
            # if the motor position is unknown -> disable and abort
            # if self.counts == -1:
            #     self._disable("Motor position unknown, recalibrate independently")
            #     return asyncio.sleep(0) #/ hacky way to make asyncio happy

            # otherwise -> execute function
            return f(self, *args, **kwargs)
        return wrapper

    async def _find_home(self):
        """Find the home position from an unknown starting position"""

        # disabled handling
        if self.disabled or self.dead:
            return

        log.info(self._clm("Find Home", message="Finding home"))

        if config.get('skip_find_home'):
            log.warning(self._clm("Find Home", message="Skipping find home"))
            self._set_home_state()
            return
        
        # use uncalibrated unless throttle is set
        throttle = None if self.throttle_up else config.get('uncalibrated_up_throttle')

        # move up as much as possible and check for stall
        exception, _, _ =  await self.move(n_counts=config.get('max_counts'), direction=config.get('up'), throttle=throttle, disable_on_exceptions=[MoveException.TIMED_OUT])

        # timed out or no stall -> failed to find home
        if exception == MoveException.TIMED_OUT or exception is None:
            log.error(self._clm("Find Home", message="Failed to find home"))
            self._disable("Failed to find home")
            return

        # set home state after motor is settled
        await asyncio.sleep(3) 
        self._set_home_state()

        log.success(self._clm("Find Home", message="Home found"))

    @_handle_disabled #/ should never be called when disabled but just in case
    async def set(self, direction: int, throttle: Optional[float] = None):
        """
        Start the motor with a specific throttle and direction

        Parameters:
            throttle (float, optional): throttle to set, defaults to None
            direction (int): direction to set

        Permutations:
            direction only -> use directional calibrated throttle (if present) or offset throttle
            direction with throttle -> set throttle
        """
        # check for valid direction
        if direction not in [config.get('up'), config.get('down')]:
            raise ValueError("Direction must be up or down")

        self.direction = direction # set direction for counts

        # specific value -> set throttle to that value
        if throttle is not None:
            if not (-1 <= throttle <= 1):
                raise ValueError("Throttle must be between -1 and 1")
            self.servo.throttle = throttle
            return
        
        # neutral positions must be calibrated and set for offset throttle
        if self.lower_neutral is None and self.upper_neutral is None:
            raise ValueError("Neutral positions not found, can't set offset throttle")

        # set calibrated throttle if possible
        if self.throttle_down and self.throttle_up:
            self.servo.throttle = self.throttle_down if direction == config.get('down') else self.throttle_up
            return

        # otherwise -> set offset throttle based on neutral and direction
        neutral = self.lower_neutral if direction == config.get('up') else self.upper_neutral
        self.servo.throttle = neutral + (direction * config.get('throttle_offset'))

    def stop(self):
        """Stop the motor"""
        log.info(self._clm("Stop"))
        self.servo._pwm_out.duty_cycle = 0  

    @_handle_disabled
    async def to_home(self, throttle: Optional[float] = None):
        """
        Move the motor to the home position

        Parameters:
            throttle (float, optional): throttle to move at, defaults to None

        Returns:
            tuple(bool, float): timed_out, time_elapsed
        """
        log.info(self._clm("To Home", message="Moving Home"))

        # already home -> return early
        if self._is_home(): 
            log.success(self._clm("To Home", message="Motor already at home"))
            return

        # move to 0 count position with default throttle
        log.info(self._clm("To Home", message="Moving Home"))

        # ignore stall detection for home position
        await self.to(target=0.0, throttle=throttle, disable_on_exceptions=[MoveException.TIMED_OUT])

    @_handle_disabled
    async def move(
        self, 
        n_counts: int,
        direction: int,
        throttle: Optional[float] = None,
        timeout: int = config.get('general_timeout'),
        disable_on_exceptions: list[MoveException] = [MoveException.TIMED_OUT, MoveException.STALLED],
    ) -> tuple[Union[MoveException, None], float, list[float]]:
        """
        Move the motor a specific number of counts at a specific throttle
        
        Parameters:
            n_counts (int): number of counts to move
            direction (int): direction to move in
            throttle (float, optional): throttle to move at, defaults to None
            timeout (int, optional): max time to move, defaults to config.get('to_position_timeout')

        Returns:
            tuple(MoveException | None, float, list[float]): exception, time_elapsed, cps_readings
        """
        max_counts: int = config.get('max_counts')
        
        # ensure n_counts is within bounds
        if n_counts > max_counts:
            raise ValueError("Counts must be less than max counts")
        if n_counts < 0:
            raise ValueError("Counts must be greater than 0")

        log.info(self._clm("Move", counts=f"{self.counts} -> {self.counts + n_counts * direction}", throttle=throttle))

        exception: Union[MoveException, None] = None # store exception if any
        start_time = time.time() # track total time
        start_counts = self.counts # track start position
        cps_readings: list[float] = [] # store cps readings for stall detection and average
        calibrated_cps = self.cps_down if direction == config.get('down') else self.cps_up

        self._start_measuring_cps() # start down measuring cps
        prev_measured_cps = self.current_cps # store previous measured cps to track change

        await self.set(direction=direction, throttle=throttle) # start motor

        while True:
            # check if the motor has reached the target position
            if abs(self.counts - start_counts) == n_counts:
                log.success(self._clm("Move", message="Motor has reached target position"))
                break
            
            # no reading before timeout -> stop and exit
            if self.last_read_time is None and time.time() - start_time > timeout:
                log.error(self._clm("Move", message="Motor timed out"))
                exception = MoveException.TIMED_OUT
                break
            
            # cps has changed -> add to readings and check stall
            if (prev_measured_cps != self.current_cps):
                # check for stall if readings are available
                if self.last_read_time is not None:
                    measured_time = 1 / self.current_cps # measured time for current cps
                    allowable_time = (1 / calibrated_cps) * 1.1 if calibrated_cps else config.get('default_allowable_time')

                    # less than 2 readings -> increase allowable time (account for acceleration)
                    if(len(cps_readings) < 2):
                        allowable_time = allowable_time * 1.5

                    # measured time exceeds max read time -> stall detected
                    if measured_time > config.get('max_read_time'):
                        log.error(self._clm("Move", message="Stall detected", reason="Measured time exceeded max read time"))
                        exception = MoveException.STALLED
                        break
                
                    # measured time exceeds allowable time -> stall detected
                    if measured_time > allowable_time:
                        log.error(self._clm("Move", message="Stall detected", reason="Measured time exceeded allowable time"))
                        exception = MoveException.STALLED
                        break

                cps_readings.append(self.current_cps)
                prev_measured_cps = self.current_cps

            await asyncio.sleep(0.01) # yield control back to event

        if exception is not None and exception in disable_on_exceptions:
            self._disable(exception.value)
        
        time_elapsed = time.time() - start_time
        self._stop_measuring_cps() # stop measuring cps
        self.stop()

        return exception, time_elapsed, cps_readings
    
    @_handle_disabled
    async def to(
            self, 
            target: float, 
            throttle: Optional[float] = None,
            timeout: int = config.get('general_timeout'),
            disable_on_exceptions: list[MoveException] = [MoveException.TIMED_OUT, MoveException.STALLED],
    ) -> tuple[Union[MoveException, None], float, list[float]]:
        """
        Move the motor to a specific position scaled between 0 and 1
        
        Parameters:
            target (float): target position to move to, between 0 and 1
            throttle (float, optional): throttle to move at, defaults to None
            direction (int, optional): direction to move in, defaults to None
            timeout (int, optional): max time to move, defaults to config.get('to_position_timeout')

        Returns:
            tuple(bool, float, list[float]): timed_out, time_elapsed, cps_readings
        """
        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        max_counts: int = config.get('max_counts')
        target_counts = int(target * max_counts)
        n_counts = target_counts - self.counts
        direction = config.get('up') if n_counts < 0 else config.get('down')
        n_counts = abs(n_counts) # use absolute value

        log.info(self._clm("To", message=f"({self.counts} -> {target_counts})", throttle=throttle))
        return await self.move(n_counts, direction, throttle, timeout, disable_on_exceptions) # move to target position

    async def recover(self):
        """Attempt to recover from a disabled state"""

        # if motor is not disabled or dead -> do nothing
        if not self.disabled or self.dead:
            return

        # if motor has attempted to recover twice -> mark as dead
        if self.recover_attempts >= config.get('max_recovery_attempts'):
            self.dead = True
            return

        self.recover_attempts += 1 # increment recover attempts
        self.disabled = False # temporarily enable motor

        # move down to verify down movement is working
        down_exception, _, _ = await self.move(direction=config.get('down'), n_counts=config.get('recovery_counts'))

        # exception -> disable motor
        if down_exception:
            self._disable("Failed to recover")

        # move up to verify up movement is working
        up_exception, _, _ = await self.move(direction=config.get('up'), n_counts=config.get('recovery_counts'))

        # exception -> disable motor
        if up_exception:
            self._disable("Failed to recover")

        # success -> leave motor enabled and reset recover attempts
        self.recover_attempts = 0
    
    # -------------------------------- CALIBRATION ------------------------------- #
    @_handle_disabled 
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

        await self.move(n_counts=3, direction=config.get('down')) # move to buffer position (avoid hitting top on up)

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
                cps: float,
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
            error = target_cps - cps # calculate error (negative if cps is greater than target, positive if less)
            new_factor = factor 

            log.info(self._clm("CRT", direction=f"{'down' if is_down else 'up'}", cps=cps, error=error))

            # throttle is within error margin or factor is really low -> found throttle -> exit 
            if abs(error) <= error_margin or factor < 0.05:
                if factor < 0.05:
                    log.warning(self._clm("CRT", message="Factor is too low, taking current throttle"))
                log.success(self._clm("CRT", message="Throttle found", throttle=throttle))
                found_throttle = True

                return cps, throttle, factor, found_throttle

            # target in between previous and current cps -> reduce scale factor
            if previous_cps < target_cps < cps or previous_cps > target_cps > cps:
                new_factor = factor * 0.75 #/ should be proportional to error
                log.info(self._clm("CRT", message=f"Reducing {'down' if is_down else 'up'} factor ({factor} -> {new_factor})"))

            # adjust throttle based on error and direction
            if is_down:
                # cps is too high -> decrease throttle
                if cps > target_cps:
                    new_throttle = throttle - (new_factor * step_size)
                # cps is too low -> increase throttle
                else:
                    new_throttle = throttle + (new_factor * step_size)
            else:
                # cps is too high -> decrease throttle
                if cps > target_cps:
                    new_throttle = throttle + (new_factor * step_size)
                # cps is too low -> increase throttle
                else:
                    new_throttle = throttle - (new_factor * step_size)

            # check if throttle is within safe neutral bounds, if not -> set original throttle and reduce factor
            if is_down and new_throttle <= cast(float, self.upper_neutral): # apply padding to upper neutral
                log.info(self._clm("CRT", message="Throttle within upper neutral bounds, setting original throttle"))
                new_throttle = throttle 
                new_factor = factor * 0.90
            if not is_down and new_throttle >= cast(float, self.lower_neutral): # apply padding to lower neutral
                log.info(self._clm("CRT", message="Throttle within lower neutral bounds, setting original throttle"))
                new_throttle = throttle
                new_factor = factor * 0.90

            log.info(self._clm("CRT", message=f"{'Down' if is_down else 'Up'} throttle ({throttle} -> {new_throttle})", factor=new_factor))

            return cps, new_throttle, new_factor, found_throttle

        # move to calibration position and measure cps until within error
        while not found_down_throttle or not found_up_throttle:
            down_cps, up_cps = await self._measure_cps(
                n_counts=config.get('calibration_counts'), 
                down_throttle=down_throttle,
                up_throttle=up_throttle,
            )

            # not found down throttle -> measure and adjust
            if not found_down_throttle:
                previous_down_cps, down_throttle, down_factor, found_down_throttle = measure_and_adjust_throttle(
                    direction=config.get('down'), 
                    cps=down_cps,
                    throttle=down_throttle, 
                    previous_cps=previous_down_cps, 
                    factor=down_factor
                )

            # not found up throttle -> measure and adjust
            if not found_up_throttle:
                previous_up_cps, up_throttle, up_factor, found_up_throttle = measure_and_adjust_throttle(
                    direction=config.get('up'), 
                    cps=up_cps, 
                    throttle=up_throttle, 
                    previous_cps=previous_up_cps, 
                    factor=up_factor
                )

        self.throttle_down = down_throttle
        self.throttle_up = up_throttle
        log.success(self._clm("CRT", throttle_down=self.throttle_down, throttle_up=self.throttle_up))

    @_handle_disabled
    async def _measure_cps(
            self, 
            n_counts: int, 
            down_throttle: Optional[float] = None, 
            up_throttle: Optional[float] = None,
            use_initial_buffer = False, 
            move_home_prior = False
        ) -> tuple[float, float]:
        """
        Measure cps for a specific number of counts and throttle
        
        Parameters:
            counts (int): number of counts to measure
            throttle (float, optional): throttle to measure at, defaults to None

        Returns:
            tuple(float): cps_down, cps_up
        """ 

        if(n_counts < 6):
            raise ValueError("Counts must be greater than 5")
        
        cps_readings_buffer = 2 # padding for cps readings (ignores acceleration)
        
        # move to home position if not already
        if move_home_prior and not self._is_home():
                await self.to_home(throttle=config.get('uncalibrated_up_throttle'))

        # move to buffer position if needed
        if use_initial_buffer:
            await self.move(n_counts=4, direction=config.get('down'))

        # ----------------------------- measure cps down ----------------------------- #
        log.info(self._clm("Measure CPS", message="Measuring cps down"))

        # move to down to calibration position and measure time (automatically stops measuring cps when done)
        down_exception, _, down_cps_readings = await self.move(n_counts=n_counts, throttle=down_throttle, direction=config.get('down')) 

        if down_exception:
            log.error(self._clm("Measure CPS", message="Exception measuring down cps", exception=down_exception))
            raise ValueError("Exception measuring down cps")

        # compute average cps down (ignoring leading and trailing readings)
        down_buffered_readings = down_cps_readings[cps_readings_buffer: len(down_cps_readings) - cps_readings_buffer]
        cps_down = sum(down_buffered_readings) / (len(down_cps_readings) - 2 * cps_readings_buffer)

        # ------------------------------ measure cps up ------------------------------ #
        log.info(self._clm("Measure CPS", message="Measuring cps up"))

        # move to up to calibration position and measure time
        up_exception, _, up_cps_readings = await self.move(n_counts=n_counts, throttle=up_throttle, direction=config.get('up'))

        if up_exception:
            log.error(self._clm("Measure CPS", message="Exception measuring up cps", exception=up_exception))
            raise ValueError("Exception measuring up cps")
        
        # compute average up down
        up_buffered_readings = up_cps_readings[cps_readings_buffer: len(up_cps_readings) - cps_readings_buffer]
        cps_up = sum(up_buffered_readings) / (len(up_cps_readings) - 2 * cps_readings_buffer)

        log.info(self._clm("Measure CPS", cps_down=cps_down, cps_up=cps_up))

        return (cps_down, cps_up)


    @_handle_disabled #/ should never be called when disabled but just in case
    async def _find_cps(self):
        """Find counts per second of the motor in both directions"""
        log.info(self._clm("Find CPS", message="Finding cps up and down"))

        # neutral positions must be calibrated and set for present throttles
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")

        # measure cps for both directions
        self.cps_down, self.cps_up = await self._measure_cps(n_counts=config.get('calibration_counts'), use_initial_buffer=True, move_home_prior=True)

        log.success(self._clm("Find CPS", cps_down=self.cps_down, cps_up=self.cps_up))

    @_handle_disabled #/ should never be called when disabled but just in case
    async def _find_neutrals(self):
        """Find the lower and upper neutral positions of motor"""
        log.info(self._clm("Find Neutrals", message="Finding neutral positions"))

        # ensure motor is at home position
        if not self._is_home():
            await self.to_home(throttle=config.get('uncalibrated_up_throttle'))

        step = 0.01 # step size for throttle
        current_throttle = 0.32 # initial throttle 
        
        # continually decrease throttle until both neutral positions are found
        while self.upper_neutral is None or self.lower_neutral is None:
            current_throttle = round(current_throttle - step, 2)
            log.info(self._clm("Find Neutrals", current_throttle=current_throttle))
            direction = config.get('down') if self.upper_neutral is None else config.get('up') # move in opposite direction of whichever neutral is not found

            exception, _, _ = await self.move(n_counts=2, direction=direction, throttle=current_throttle, timeout=config.get("calibrate_neutral_timeout"), disable_on_exceptions=[MoveException.STALLED])

            # initial throttle has timed out -> found upper neutral
            if self.upper_neutral is None and exception == MoveException.TIMED_OUT:
                log.info(self._clm("Find Neutrals", message=f"Upper neutral found: {current_throttle}"))
                self.upper_neutral = current_throttle

            # upper neutral found and motor has not timed out -> found lower neutral
            if self.lower_neutral is None and not exception and self.upper_neutral is not None:
                log.info(self._clm("Find Neutrals", message=f"Lower neutral found: {current_throttle}"))
                self.lower_neutral = current_throttle + step # add step to account for last iteration

        log.info(self._clm("Find Neutrals", lower_neutral=self.lower_neutral, upper_neutral=self.upper_neutral))

    async def calibrate_independent(self):
        """Calibrate motors independent variables by finding neutral positions and cps in both directions"""
        log.info(self._clm("Calibrate Independent", message="Calibrating Motor"))

        # find initial home position if not already found
        if self.counts == -1:
            await self._find_home()

        # find neutrals if either is not present
        if self.lower_neutral is None or self.upper_neutral is None:
            await self._find_neutrals()

        # find cps if either is not present
        if self.cps_down is None or self.cps_up is None:
            await self._find_cps()

        await self.to_home() # move back to home position for next calibration