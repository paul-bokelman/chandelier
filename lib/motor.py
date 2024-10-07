from typing import Optional, Union
from store import SingularCalibrationData
from adafruit_servokit import ContinuousServo
import time
import asyncio
import RPi.GPIO as GPIO
from lib.utils import log, to_seconds
from configuration.config import config

# todo: all disabled functionality should be present here (don't run motor if disabled)

class Motor:
    """Motor class to control a single motor"""
    def __init__(self, channel: int, servo: ContinuousServo) -> None:
        self.channel = channel
        self.last_read_time = None
        self.encoder_feedback_disabled = False # get feedback from encoder
        self.direction = config.get('down') # direction of motor
        self.encoder_pin = config.get('encoder_pins')[self.channel]
        self.servo = servo
        self.servo.set_pulse_width_range(1000, 2000) 
        self.disabled = False

        # calibrated data
        # todo: counts should be none unless calibrated (position unknown)
        self.counts = -1  # current count position, -1 indicates that the motor has not been calibrated
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.throttle_down = None # relative throttle for moving down at slow preset
        self.throttle_up = None # relative throttle for moving up at slow preset

        self.lower_neutral = None
        self.upper_neutral = None

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

        if self.channel in config.get('initial_disabled_motors'):
            self.disabled = True
            log.warning(f"M{self.channel} | Motor disabled")

    # todo: needs overhaul in consistency and error handling
    def _encoder_callback(self, channel: int):
        """Callback function for encoder"""
        if self.encoder_feedback_disabled:
            return

        self.counts += self.direction * 1 # increment encoder count

        self.last_read_time = time.time()

        if not config.get('suppress_count_logging'):
            log.info(f"M{self.channel} | count: {self.counts} | direction: {'down' if self.direction == config.get('down') else 'up'}")

    # todo: completely useless rn...
    def _at_home(self):
        """Set motor home state"""
        self.encoder_feedback_disabled = True
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

    # todo: should be private method and maybe have additional checks?
    def is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0

    # todo: needs overhaul, no throttle -> use offset with direction
    def set(self, throttle: Throttle = config.get('ThrottlePresets.SLOW'), direction: Optional[int] = None):
        """Set a specific servo to a specific or set throttle and direction"""
        if not isinstance(throttle, (config.get('ThrottlePresets'), float)):
            raise ValueError(f"Throttle must be a float or ThrottlePresets, got {type(throttle)}")

        # specific value -> set throttle
        if isinstance(throttle, float):
            if not (-1 <= throttle <= 1):
                raise ValueError("Throttle must be between -1 and 1")
            
            self.servo.throttle = throttle
            return
        
        # neutral positions must be calibrated and set for present throttles
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")
        
        if direction not in [config.get('up'), config.get('down')]:
            raise ValueError("Direction must be up or down for preset throttle")
        if throttle not in config.get('ThrottlePresets'):
            raise ValueError("Throttle must be a valid preset")
        
        # only supports slow speed for now
        if throttle not in [config.get('ThrottlePresets.SLOW')]: 
            raise ValueError("Throttle must be relatively calibrated")
        
        # set throttle based on preset (relative to neutral)
        offset = throttle.value

        # set throttle based on direction and calibrated relative throttles (fallback to neutral if not set)
        if direction == config.get('up'):
            # calibrated throttle is set -> use it (only slow)
            if throttle == config.get('ThrottlePresets.SLOW') and self.throttle_up is not None:
                self.servo.throttle = self.throttle_up
            else:
                self.servo.throttle = (self.lower_neutral - offset)
        else:
            # calibrated throttle is set -> use it (only slow)
            if throttle == config.get('ThrottlePresets.SLOW') and self.throttle_down is not None:
                self.servo.throttle = self.throttle_down
            else:
                self.servo.throttle = (self.upper_neutral + offset)

    def stop(self):
        """Stop the motor"""
        log.info(self._clm("Stop"))
        self.servo._pwm_out.duty_cycle = 0  

    # todo: requires overhaul -> use counts instead of timeouts
    # todo: ensure that throttle is always negative
    async def to_home(self, throttle: Throttle = config.get('ThrottlePresets.SLOW'), override_initial_timeout = False) -> tuple[bool, float]:
        """Move the motor to the home position (0 count)"""
        if config.get('mimic_home'):
            log.info(self._clm("To Home", message="Mimicked home movement"))
            self.counts = 0
            return False, 0.0

        self.encoder_feedback_disabled = False # start incrementing encoder counts
        timed_out = False
        log.info(self._clm("To Home", message="Moving Home"))

        if self.is_home(): 
            log.success(self._clm("To Home", message="Motor already at home"))
            return timed_out, 0.0

        self.set(throttle=throttle, direction=config.get('up')if isinstance(throttle, config.get('ThrottlePresets')) else None)
        self.last_read_time = None 
        start_time = time.time()

        log.info(self._clm("To Home", message="Moving Home"))
        
        # move motor to home position and check for stalls
        while True:
            current_time = time.time()
            # initial count position has not changed -> already home or jammed
            if not override_initial_timeout:
                if current_time - start_time > config.get('to_home_initial_timeout') and self.last_read_time is None:
                    log.success(self._clm("To Home", message="Motor at home position"))
                    self._at_home()
                    break
            # check if the motor has reached home
            if self.last_read_time is not None and current_time - self.last_read_time > config.get('to_home_max_interval'):
                log.success(self._clm("To Home", message="Motor at home position", time=to_seconds(time.time() - start_time)))
                self._at_home()
                break
            # if the motor has timed out, stop the motor
            if current_time - start_time > config.get('to_home_timeout'):
                self._disable("Motor timed out, returning home")
                timed_out = True
                break
            
            await asyncio.sleep(0.01) # yield control back to event

        self.stop()
        return timed_out, (time.time() - start_time)
    
    # todo: move counts, needs work and cleanup
    async def move(
        self, 
        n_counts: int,
        throttle: Throttle = config.get('ThrottlePresets.SLOW'), 
        direction: Optional[int] = None,
        timeout: int = config.get('to_position_timeout'),
        ensure_enabled = False
    ) -> tuple[bool, float]:
        """Move the motor n number of counts at a specific speed"""

        max_counts = config.get('testing_max_counts') if config.get('testing_mode') else config.get('max_counts')
        
        # validate input
        if isinstance(n_counts, int) == False:
            raise ValueError("Counts must be an integer")
        if n_counts > max_counts:
            raise ValueError("Counts must be less than max counts")
        if n_counts < 0:
            raise ValueError("Counts must be greater than 0")

        timed_out = False

        if direction is not None:
            self.direction = direction
            
        log.info(self._clm("Move", start_counts=self.counts, n_counts=n_counts, direction=self.direction, throttle=throttle))
        
        if n_counts == 0:
            log.success(self._clm("Move", message="No counts to move"))
            return timed_out, 0
        
        self.encoder_feedback_disabled = False # start incrementing encoder counts
        self.set(throttle, direction)

        start_time = time.time() # track time
        start_counts = self.counts

        prev_counts = self.counts
        last_encoder_time = time.time()
        
        while True:
            # check if the motor has reached the target position (have to check for abs because direction may be unknown)
            if abs(self.counts - start_counts) == n_counts:
                log.success(self._clm("Move", message="Motor has reached target position"))
                break

            # encoder has not been triggered -> motor is jammed or sensor is not working properly
            if not ensure_enabled:
                # increment encoder reading
                if prev_counts != self.counts:
                    prev_counts = self.counts
                    last_encoder_time = time.time()
                # encoder has not been triggered for a long time -> disable motor and exit
                if last_encoder_time is not None and time.time() - last_encoder_time > config.get('max_time_between_encoder_readings'):
                    self._disable("Encoder readings too far apart")
                    timed_out = True
                    break
            # hasn't reached target position before timeout -> exit
            if time.time() - start_time > timeout:
                if not ensure_enabled:
                    self._disable("Motor timed out")
                else:
                    log.error(self._clm("Move", message="Motor timed out"))
                timed_out = True
                break

            await asyncio.sleep(0.01) # yield control back to event

        time_elapsed = time.time() - start_time

        log.info(self._clm("Move", message=f"Time Elapsed: {time_elapsed}"))

        self.encoder_feedback_disabled = True # stop incrementing encoder counts
        self.stop()
        return timed_out, time_elapsed
    
    # todo: should just use move() with util conversion of position to relative counts
    async def to(
            self, target: float, 
            throttle: Throttle = config.get('ThrottlePresets.SLOW'),
            timeout: int = config.get('to_position_timeout'),
    ) -> tuple[bool, float]:
        """Move the motor to a specific position relative to `max_counts` at a specific speed"""
        log.info(self._clm("To"))

        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        max_counts = config.get('testing_max_counts') if config.get('testing_mode') else config.get('max_counts')
        
        target_counts = int(target * max_counts)

        log.info(self._clm("To", message=f"({self.counts} -> {target_counts})", throttle=throttle))

        n_counts = target_counts - self.counts

        self.direction = config.get('up') if n_counts < 0 else config.get('down')
        return await self.move(abs(n_counts), throttle, self.direction, timeout) # move to target position
    
    # -------------------------------- CALIBRATION ------------------------------- #
    async def calibrate_relative_throttles(self, target_up_cps: float, target_down_cps: float):
        """Find relative throttles based on global cps data (only configured for slow speed)"""
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
        if not self.is_home():
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
            """Measure cps and adjust throttle based on error for a specific direction"""
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
            down_timed_out, down_time_elapsed = await self.move(n_counts=config.get('calibration_counts'), throttle=down_throttle, timeout=config.get('calibration_timeout'))
            
            # move timed out -> exit
            if down_timed_out:
                log.error(self._clm("CRT", message="Motor timed out moving down", throttle=down_throttle, time_elapsed=down_time_elapsed))
                raise ValueError("Motor timed out moving down")
        
            # move back to previous position to measure up cps
            up_timed_out, up_time_elapsed = await self.move(n_counts=config.get('calibration_counts'), throttle=up_throttle, direction=config.get('up'), timeout=config.get('calibration_timeout'))

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
        if not self.is_home():
            await self.to_home(config.get('uncalibrated_throttle'))

        await self.to(0.1) # move to buffer position 

         # ------------------------------- find cps down ------------------------------ #
        if self.cps_down is None:
            log.info(self._clm("Find CPS", message="Finding cps down"))

            # move to down to calibration position and measure time
            timed_out, time_elapsed = await self.move(
                n_counts=config.get('calibration_counts'),
                throttle=(self.lower_neutral + config.get('throttle_offset')), # todo: min throttle should be calculated in move()
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
            timed_out, time_elapsed = await self.move(
                n_counts=config.get('calibration_counts'),
                throttle=(self.upper_neutral + config.get('throttle_offset')), # todo: min throttle should be calculated in move()
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
        if not self.is_home():
            await self.to_home(config.get('uncalibrated_throttle'))

        step = 0.01 
        current_throttle = 0.35
        
        # continually decrease throttle until both neutral positions are found
        while self.upper_neutral is None or self.lower_neutral is None:
            current_throttle = round(current_throttle - step, 2)
            log.info(self._clm("Find Neutrals", current_throttle=current_throttle))
            timed_out, _ = await self.move(n_counts=2, throttle=current_throttle, timeout=config.get("calibration_timeout"), ensure_enabled=True)

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

        self.encoder_feedback_disabled = False # start incrementing encoder counts

        # find neutrals if either is not present
        if self.lower_neutral is None or self.upper_neutral is None:
            await self._find_neutrals()

        # find cps if either is not present
        if self.cps_down is None or self.cps_up is None:
            await self._find_cps()

        await self.to_home() # move back to home position for next calibration
        
        self.encoder_feedback_disabled = True
        
    def get_calibration_data(self) -> SingularCalibrationData:
        """Get calibration data for this motor"""
        return {
            "cps_down": self.cps_down,
            "cps_up": self.cps_up,
            "lower_neutral": self.lower_neutral,
            "upper_neutral": self.upper_neutral,
            "throttle_down": self.throttle_down,
            "throttle_up": self.throttle_up
        }