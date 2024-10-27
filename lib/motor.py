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

class Status(Enum):
    """Status enum for motor"""
    ENABLED = 0
    DISABLED = 1
    DEAD = 2

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, channel: int, servo: ContinuousServo) -> None:
        # general data
        self.channel = channel
        self.direction: int = config.get('down') # direction of motor (used in encoder callback)
        self.servo = servo

        # state data
        self.status = Status.ENABLED
        self.recover_attempts = 0 # number of times the motor has attempted to recover

        # encoder data
        self.encoder_pin = config.get('encoder_pins')[self.channel]
        self.counts = -1  # current count position, -1 indicates that the motor position is unknown
        self.measuring_cps = False # if the encoder is currently being read
        self.last_read_time: Union[float, None] = None # previous time the encoder was read

        # calibration data
        self.lower_neutral = None
        self.upper_neutral = None
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.throttle_down = None # relative throttle for moving down at slow preset
        self.throttle_up = None # relative throttle for moving up at slow preset

        # detect when encoder is triggered (matches 1->0)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

        # set motors initial status based on configuration
        if self.channel in config.get('disabled_motors'):
            self.status = Status.DISABLED
            log.warning(self._clm("init", status="Motor is disabled"))
        if self.channel in config.get('dead_motors'):
            self.status = Status.DEAD
            log.warning(self._clm("init", status="Motor is dead"))

    def _encoder_callback(self, _):
        """Callback function for encoder"""
        self.counts += self.direction * 1 # increment or decrement counts based on direction

        if self.measuring_cps:
            self.last_read_time = time.time() # update previous read time

        # log counts and direction
        if not config.get('suppress_count_logging'):
            log.info(f"M{self.channel} | count: {self.counts} | direction: {'down' if self.direction == config.get('down') else 'up'}")

    def _start_measuring_cps(self):
        """Start measuring cps"""
        self.last_read_time = None # reset reading
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
    
    def disable(self, reason: str = "Unknown"):
        """
        Disable the motor
        
        Parameters:
            reason (str, optional): reason for disabling, defaults to "Unknown"
        """
        log.error(self._clm("Disable", message="Disabling motor", reason=reason))

        # set status to disabled if not already dead
        if self.status is not Status.DEAD:
            self.status = Status.DISABLED
        else:
            log.warning(self._clm("Disable", message="Motor is already dead"))

    def enable(self):
        """Enable the motor"""
        log.info(self._clm("Enable", message="Enabling motor"))

        # set status to enabled if not already dead
        if self.status is not Status.DEAD:
            self.status = Status.ENABLED
        else:
            log.warning(self._clm("Enable", message="Motor is dead"))

    def _is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0
    
    def _is_calibrated(self) -> bool:
        """Check if the motor is calibrated"""
        
        # neutrals not calibrated
        if self.lower_neutral is None or self.upper_neutral is None:
            log.info(self._clm("Calibrated", message="Calibrated neutral positions not found"))
            return False
        
        # cps not calibrated
        if self.cps_down is None or self.cps_up is None:
            log.info(self._clm("Calibrated", message="Calibrated cps not found"))
            return False
        
        if self.throttle_down is None or self.throttle_up is None:
            log.info(self._clm("Calibrated", message="Calibrated throttle not found"))
            return False
        
        return True

    async def find_home(self):
        """Find the home position from an unknown starting position"""

        log.info(self._clm("Find Home", message="Finding home"), override=True)

        if config.get('skip_find_home'):
            log.warning(self._clm("Find Home", message="Skipping find home"))
            self._set_home_state()
            return
        
        # use uncalibrated unless throttle is set
        throttle = None if self.throttle_up else config.get('uncalibrated_up_throttle')

        # move up as much as possible and check for stall
        stalled, _ =  await self.move(n_counts=(2 * config.get('max_counts')), direction=config.get('up'), throttle=throttle, disable_on_stall=False)

        # didn't stall -> failed to find home
        if not stalled:
            log.error(self._clm("Find Home", message="Max counts reached before finding home"))
            self.disable("Max counts reached before finding home")
            return

        await asyncio.sleep(2) # wait for motor to settle (background processes can still run**)
        self._set_home_state() # set home state after motor is settled

        log.success(self._clm("Find Home", message="Home found"), override=True)

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

    async def to_home(self, throttle: Optional[float] = None):
        """
        Move the motor to the home position

        Parameters:
            throttle (float, optional): throttle to move at, defaults to None

        Returns:
            tuple(bool, list[float]): stalled, cps_readings
        """
        log.info(self._clm("To Home", message="Moving Home"), override=True)

        # already home -> return early
        if self._is_home(): 
            log.success(self._clm("To Home", message="Motor already at home"), override=True)
            return

        # ignore stall detection for home position
        await self.to(target=0.0, throttle=throttle, disable_on_stall=False)

    async def move(
        self, 
        n_counts: int,
        direction: int,
        throttle: Optional[float] = None,
        disable_on_stall = True
    ) -> tuple[bool, list[float]]:
        """
        Move the motor a specific number of counts at a specific throttle
        
        Parameters:
            n_counts (int): number of counts to move
            direction (int): direction to move in
            throttle (float, optional): throttle to move at, defaults to None
            disable_on_stall (bool, optional): disable motor on stall, defaults to True

        Returns:
            tuple(bool, list[float]): stalled, cps_readings
        """
        # ensure n_counts is positive
        if n_counts < 0:
            raise ValueError("Counts must be greater than 0")

        log.info(self._clm("Move", counts=f"{self.counts} -> {self.counts + n_counts * direction}", throttle=throttle), override=True)

        stalled = False
        cps_readings: list[float] = [] # store cps readings for stall detection and average
        start_counts = self.counts # track start position

        # direction switches -> remove count if encoder is low
        if self.direction != direction and GPIO.input(self.encoder_pin) == GPIO.LOW:
            n_counts += 1

        await self.set(direction=direction, throttle=throttle) # start motor
        self._start_measuring_cps() # start measuring cps
        prev_read_time = time.time() # track previous read time

        while True:
            count_diff = abs(self.counts - start_counts)
            # check if the motor has reached the target position
            if count_diff == n_counts:
                log.success(self._clm("Move", message="Motor has reached target position"), override=True)
                break

            # ------------------------------ stall detection ----------------------------- #

            # calculate allowable and measured time based on data
            allowable_time: float = 1 / (config.get('default_allowable_down_cps') if direction == config.get('down') else config.get('default_allowable_up_cps')) # default allowable time (used for 0->2, and n-2->n counts)

            measured_time = time.time() - prev_read_time

            # new reading -> update stall information
            if self.last_read_time is not None and (prev_read_time != self.last_read_time):
                log.info(self._clm("Move", cps=round(1 / measured_time, 3))) # log cps reading
                cps_readings.append(1 / measured_time) # add cps reading
                prev_read_time = self.last_read_time # update previous read time

            # more than 2 reads & before last 2 reads -> calculate allowable time to be average of previous cps values
            if len(cps_readings) >= 2 and n_counts - count_diff > 2:
                # calculate allowable time based on all average of all previous cps readings, excluding leading
                average_cps = sum(cps_readings[1:]) / len(cps_readings[1:])
                allowable_time = (1 / average_cps) * 1.7 # add 70% buffer (account for acceleration)

            # time between readings exceeds allowable time -> stall detected
            if measured_time > allowable_time:
                log.error(self._clm("Move", message="Stall detected", mt=round(measured_time, 4), at=round(allowable_time, 4)))

                if disable_on_stall:
                    self.disable("Stalled")

                stalled = True
                break

            await asyncio.sleep(0.01) # yield control back to event

        self._stop_measuring_cps() # stop measuring cps
        self.stop()

        return stalled, cps_readings
    
    async def to(
            self, 
            target: float, 
            throttle: Optional[float] = None,
            disable_on_stall = True,
    ) -> tuple[bool, list[float]]:
        """
        Move the motor to a specific position scaled between 0 and 1
        
        Parameters:
            target (float): target position to move to, between 0 and 1
            throttle (float, optional): throttle to move at, defaults to None
            direction (int, optional): direction to move in, defaults to None
            disable_on_stall (bool, optional): disable motor on stall, defaults to False

        Returns:
            tuple(bool, list[float]): stalled, cps_readings
        """
        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        max_counts: int = config.get('max_counts')
        target_counts = int(target * max_counts)
        n_counts = target_counts - self.counts
        direction = config.get('up') if n_counts < 0 else config.get('down')
        n_counts = abs(n_counts) # use absolute value

        log.info(self._clm("To", message=f"({self.counts} -> {target_counts})", throttle=throttle))
        return await self.move(n_counts, direction, throttle, disable_on_stall) # move to target position

    async def recover(self):
        """Attempt to recover from a disabled state"""
        log.info(self._clm("Recover", message="Attempting to recover"), override=True)

        # motor is enabled or dead -> return early and count attempt (should never happen)
        if self.status != Status.DISABLED:
            log.error(self._clm("Recover", message=f"Attempted to recover {self.status.name} motor"))
            self.recover_attempts += 1
            return

        # if motor has attempted to recover twice -> mark as dead
        if self.recover_attempts >= config.get('max_recovery_attempts'):
            self.status = Status.DEAD
            return
        
        # motor is not calibrated -> return early and count attempt 
        if not self._is_calibrated():
            log.info(self._clm("Recover", message="Motor is not calibrated, skipping recovery"))
            self.recover_attempts += 1
            return

        self.recover_attempts += 1 # increment recover attempts

        # move down to verify down movement is working
        up_stall, _ = await self.move(direction=config.get('down'), n_counts=config.get('recovery_counts'))

        # stalled on up -> disable motor
        if up_stall:
            log.info(self._clm("Recover", message="Failed to recover (stalled on up)"))
            return

        # move up to verify up movement is working
        down_stall, _ = await self.move(direction=config.get('up'), n_counts=config.get('recovery_counts'))

        # stalled on down -> disable motor
        if down_stall:
            log.info(self._clm("Recover", message="Failed to recover (stalled on down)"))
            return

        await self.find_home() # recalibrate home position

        # success -> leave motor enabled and reset recover attempts
        self.recover_attempts = 0
        self.status = Status.ENABLED

        log.success(self._clm("Recover", message="Recovery successful"), override=True)
    
    # -------------------------------- CALIBRATION ------------------------------- #
    async def calibrate_relative_throttles(self, target_up_cps: float, target_down_cps: float):
        """
        Find and assign relative throttles based on global cps data (only configured for slow speed)
        
        Parameters:
            target_up_cps (float): target cps moving up
            target_down_cps (float): target cps moving
        """
        
        # pre-checks
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")
        if self.cps_down is None or self.cps_up is None:
            raise ValueError("CPS not found")

        if self.throttle_down is not None and self.throttle_up is not None:
            log.info(self._clm("CRT", message="Throttles already calibrated"))
            return
        
        log.info(self._clm("CRT", target_down_cps=target_down_cps, target_up_cps=target_up_cps))

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
                cps (float): current cps
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

            # requested throttle within neutral bounds -> set original throttle and reduce factor
            if new_throttle >= cast(float, self.lower_neutral) and new_throttle <= cast(float, self.upper_neutral):
                log.info(self._clm("CRT", message="Throttle within neutral bounds, setting original throttle", throttle=throttle))
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

        log.success(self._clm("CRT", throttle_down=self.throttle_down, throttle_up=self.throttle_up), override=True)

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
        down_stall, down_cps_readings = await self.move(n_counts=n_counts, throttle=down_throttle, direction=config.get('down')) 

        if down_stall:
            log.error(self._clm("Measure CPS", message="Stalled measuring down cps"))
            raise ValueError("Stalled measuring down cps")

        # compute average cps down (ignoring leading and trailing readings)
        down_buffered_readings = down_cps_readings[cps_readings_buffer: len(down_cps_readings) - cps_readings_buffer]
        cps_down = sum(down_buffered_readings) / (len(down_cps_readings) - 2 * cps_readings_buffer)

        # ------------------------------ measure cps up ------------------------------ #
        log.info(self._clm("Measure CPS", message="Measuring cps up"))

        # move to up to calibration position and measure time
        up_stall, up_cps_readings = await self.move(n_counts=n_counts, throttle=up_throttle, direction=config.get('up'))

        if up_stall:
            log.error(self._clm("Measure CPS", message="Stalled measuring up cps"))
            raise ValueError("Stalled measuring up cps")
        
        # compute average up down
        up_buffered_readings = up_cps_readings[cps_readings_buffer: len(up_cps_readings) - cps_readings_buffer]
        cps_up = sum(up_buffered_readings) / (len(up_cps_readings) - 2 * cps_readings_buffer)

        log.info(self._clm("Measure CPS", cps_down=cps_down, cps_up=cps_up))

        return (cps_down, cps_up)


    async def _find_cps(self):
        """Find counts per second of the motor in both directions"""
        log.info(self._clm("Find CPS", message="Finding cps up and down"), override=True)

        # neutral positions must be calibrated and set for present throttles
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")

        # measure cps for both directions
        self.cps_down, self.cps_up = await self._measure_cps(n_counts=config.get('calibration_counts'), use_initial_buffer=True, move_home_prior=True)

        log.success(self._clm("Find CPS", cps_down=self.cps_down, cps_up=self.cps_up), override=True)

    async def _find_neutrals(self):
        """Find the lower and upper neutral positions of motor"""
        log.info(self._clm("Find Neutrals", message="Finding neutral positions"), override=True)

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

            # move motor and check for stall
            stalled, _ = await self.move(n_counts=2, direction=direction, throttle=current_throttle, disable_on_stall=False)

            # initial throttle has timed out -> found upper neutral
            if self.upper_neutral is None and stalled:
                log.success(self._clm("Find Neutrals", message=f"Upper neutral found: {current_throttle}"))
                self.upper_neutral = current_throttle

            # upper neutral found and motor has not timed out -> found lower neutral
            if self.lower_neutral is None and not stalled and self.upper_neutral is not None:
                log.success(self._clm("Find Neutrals", message=f"Lower neutral found: {current_throttle}"))
                self.lower_neutral = current_throttle + step # add step to account for last iteration

        # apply buffer to neutral positions
        self.lower_neutral = self.lower_neutral - step 
        self.upper_neutral = self.upper_neutral + step 
        log.success(self._clm("Find Neutrals", lower_neutral=self.lower_neutral, upper_neutral=self.upper_neutral), override=True)

    async def calibrate_independent(self):
        """Calibrate motors independent variables by finding neutral positions and cps in both directions"""
        log.info(self._clm("Calibrate Independent", message="Calibrating Motor"), override=True)

        # find initial home position if not already found
        if self.counts == -1:
            await self.find_home()

        # find neutrals if either is not present
        if self.lower_neutral is None or self.upper_neutral is None:
            await self._find_neutrals()

        # find cps if either is not present
        if self.cps_down is None or self.cps_up is None:
            await self._find_cps()

        if not self._is_home():
            await self.to_home() # move back to home position for next calibration