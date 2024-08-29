from typing import Optional, Union
from adafruit_servokit import ContinuousServo
import time
import asyncio
import RPi.GPIO as GPIO
from lib.store import SingularCalibrationData
from lib.utils import log, to_seconds
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
        self.disabled = False

        # calibrated data
        # todo: counts should be none unless calibrated (position unknown)
        self.counts = -1  # current count position, -1 indicates that the motor has not been calibrated
        self.cps_down: Optional[float] = None # counts per second moving down, -1 indicates that the motor has not been calibrated
        self.cps_up: Optional[float] = None # counts per second moving up, -1 indicates that the motor has not been calibrated
        self.slow_throttle_down = None # relative throttle for moving down at slow preset
        self.slow_throttle_up = None # relative throttle for moving up at slow preset

        self.lower_neutral = None
        self.upper_neutral = None

        # detect when encoder is triggered (matches 0's)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self._encoder_callback, bouncetime=2)

        if self.channel in constants.initial_disabled_motors:
            self.disabled = True
            log.warning(f"M{self.channel} | Motor disabled")

    def _encoder_callback(self, channel: int):
        """Callback function for encoder"""
        if self.encoder_feedback_disabled:
            return

        self.counts += self.direction * 1 # increment encoder count

        self.last_read_time = time.time()

        if not constants.suppress_count_logging:
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
    
    def _disable(self, reason: str = "Unknown"):
        log.error(self._clm("Disable", message="Disabling motor", reason=reason))
        self.disabled = True # todo: re-enable this

    def set(self, throttle: Throttle = constants.ThrottlePresets.SLOW, direction: Optional[int] = None):
        """Set a specific servo to a specific or set throttle and direction"""
        if not isinstance(throttle, (constants.ThrottlePresets, float)):
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
        
        if direction not in [constants.up, constants.down]:
            raise ValueError("Direction must be up or down for preset throttle")
        if throttle not in constants.ThrottlePresets:
            raise ValueError("Throttle must be a valid preset")
        
        # only supports slow speed for now
        if throttle not in [constants.ThrottlePresets.SLOW]: 
            raise ValueError("Throttle must be relatively calibrated")
        
        # set throttle based on preset (relative to neutral)
        offset = throttle.value

        # set throttle based on direction and calibrated relative throttles (fallback to neutral if not set)
        if direction == constants.up:
            # calibrated throttle is set -> use it (only slow)
            if throttle == constants.ThrottlePresets.SLOW and self.slow_throttle_up is not None:
                self.servo.throttle = self.slow_throttle_up
            else:
                self.servo.throttle = (self.lower_neutral - offset)
        else:
            # calibrated throttle is set -> use it (only slow)
            if throttle == constants.ThrottlePresets.SLOW and self.slow_throttle_down is not None:
                self.servo.throttle = self.slow_throttle_down
            else:
                self.servo.throttle = (self.upper_neutral + offset)

    def stop(self):
        """Stop the motor"""
        log.info(self._clm("Stop"))
        self.servo._pwm_out.duty_cycle = 0  

    async def to_home(self, throttle: Throttle = constants.ThrottlePresets.SLOW, override_initial_timeout = False) -> tuple[bool, float]:
        """Move the motor to the home position (0 count)"""
        if constants.mimic_home:
            log.info(self._clm("To Home", message="Mimicked home movement"))
            self.counts = 0
            return False, 0.0

        self.encoder_feedback_disabled = False # start incrementing encoder counts
        timed_out = False
        log.info(self._clm("To Home", message="Moving Home"))

        if self.is_home(): 
            log.success(self._clm("To Home", message="Motor already at home"))
            return timed_out, 0.0

        self.set(throttle=throttle, direction=constants.up if isinstance(throttle, constants.ThrottlePresets) else None)
        self.last_read_time = None 
        start_time = time.time()

        log.info(self._clm("To Home", message="Moving Home"))
        
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
                log.success(self._clm("To Home", message="Motor at home position", time=to_seconds(time.time() - start_time)))
                self._at_home()
                break
            # if the motor has timed out, stop the motor
            if current_time - start_time > constants.to_home_timeout:
                self._disable("Motor timed out, returning home")
                timed_out = True
                break
            
            await asyncio.sleep(0.01) # yield control back to event

        self.stop()
        return timed_out, (time.time() - start_time)
    
    async def move(
        self, 
        n_counts: int,
        throttle: Throttle = constants.ThrottlePresets.SLOW, 
        direction: Optional[int] = None,
        timeout: int = constants.to_position_timeout,
        ensure_enabled = False
    ) -> tuple[bool, float]:
        """Move the motor n number of counts at a specific speed"""

        max_counts = constants.testing_max_counts if constants.testing_mode else constants.max_counts
        
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
                if last_encoder_time is not None and time.time() - last_encoder_time > constants.max_time_between_encoder_readings:
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
        
    async def to(
            self, target: float, 
            throttle: Throttle = constants.ThrottlePresets.SLOW, 
            timeout: int = constants.to_position_timeout
    ) -> tuple[bool, float]:
        """Move the motor to a specific position relative to `max_counts` at a specific speed"""
        log.info(self._clm("To"))

        if target < 0 or target > 1:
            raise ValueError("Position must be between 0 and 1")
        
        max_counts = constants.testing_max_counts if constants.testing_mode else constants.max_counts
        
        target_counts = int(target * max_counts)

        log.info(self._clm("To", message=f"({self.counts} -> {target_counts})", throttle=throttle))

        n_counts = target_counts - self.counts

        self.direction = constants.up if n_counts < 0 else constants.down
        return await self.move(abs(n_counts), throttle, self.direction, timeout) # move to target position
    
    # -------------------------------- CALIBRATION ------------------------------- #
    async def find_relative_throttles(self, max_cps_up: float, max_cps_down: float):
        """Find relative throttles based on global cps data (only configured for slow speed)"""
        log.info(self._clm("FRT"))
        
        # pre-checks
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")
        if self.cps_down is None or self.cps_up is None:
            raise ValueError("CPS not found")

        if self.slow_throttle_down is not None and self.slow_throttle_up is not None:
            log.info(self._clm("FRT", message="Throttles already calibrated"))
            return

        # move back home if not already (allows _find_cps to be called separately from _find_neutrals)
        if not self.is_home():
            await self.to_home()

        await self.to(0.1) # move to buffer position (avoid hitting top on up)

        # descent configuration
        error = 0.03 # error margin
        down_step = 0.01
        up_step = 0.01

        target_down_cps = max_cps_down
        target_up_cps = max_cps_up
        previous_down_cps = None
        previous_up_cps = None
        down_throttle = self.upper_neutral + down_step 
        up_throttle = self.lower_neutral - up_step

        log.info(self._clm("FRT", target_down_cps=target_down_cps, target_up_cps=target_up_cps))

        found_relative_down_cps = lambda cps: abs(target_down_cps - cps) < error
        found_relative_up_cps = lambda cps: abs(target_up_cps - cps) < error

        # move to calibration position and measure cps until within error
        while (previous_down_cps is None or not found_relative_down_cps(previous_down_cps)) or (previous_up_cps is None or not found_relative_up_cps(previous_up_cps)):
            log.info(self._clm("FRT", message="Finding throttle", up_step=up_step, down_step=down_step))

            # move down to measure down cps
            down_timed_out, down_time_elapsed = await self.move(n_counts=constants.calibration_counts, throttle=down_throttle, timeout=constants.calibration_timeout)

            # move timed out -> exit
            if down_timed_out:
                log.error(self._clm("FRT", message="Throttle timed out moving down", throttle=down_throttle))
                return
            
            down_cps = constants.calibration_counts / down_time_elapsed # calculate cps down

            # move back to previous position to measure up cps
            up_timed_out, up_time_elapsed = await self.move(n_counts=constants.calibration_counts, throttle=up_throttle, direction=constants.up, timeout=constants.calibration_timeout)

            # to home timed out -> exit
            if up_timed_out:
                log.error(self._clm("FRT", message="Throttle timed out moving home", throttle=up_throttle))
                return
            
            # calculate cps up
            up_cps = constants.calibration_counts / up_time_elapsed

            # target in between previous and current cps -> divide step size (more granular)
            if previous_up_cps is not None and not found_relative_up_cps(up_cps):
                if previous_up_cps < target_up_cps < up_cps or previous_up_cps > target_up_cps > up_cps:
                    log.info(self._clm("FRT", message=f"Decreasing step size (UP), ({up_step} -> {up_step / 2})"))
                    up_step /= 2

            if previous_down_cps is not None and not found_relative_down_cps(down_cps):
                if previous_down_cps < target_down_cps < down_cps or previous_down_cps > target_down_cps > down_cps:
                    log.info(self._clm("FRT", message=f"Decreasing step size (DOWN), ({down_step} -> {down_step / 2})"))
                    down_step /= 2

            previous_down_cps = down_cps
            previous_up_cps = up_cps
            down_distance = target_down_cps - down_cps # calculate distance from target cps (DOWN)
            up_distance = target_up_cps - up_cps # calculate distance from target cps (UP)

            # throttle is not within error margin -> adjust throttle
            if not found_relative_down_cps(down_cps):
                # throttle too low -> increase throttle (DOWN)
                if down_cps < target_down_cps:
                    down_throttle += down_step
                    log.info(self._clm("FRT", message="Increasing (DOWN)", throttle=down_throttle, down_distance=down_distance))
                # throttle too high -> decrease throttle and decrease step size (DOWN)
                else:
                    down_throttle -= down_step
                    log.info(self._clm("FRT", message="Decreasing (DOWN)", throttle=down_throttle, down_distance=down_distance))

            # throttle is not within error margin -> adjust throttle
            if not found_relative_up_cps(up_cps):
                # throttle too low -> increase throttle (UP)
                if up_cps < target_up_cps:
                    up_throttle -= up_step
                    log.info(self._clm("FRT", message="Increasing (UP)", throttle=up_throttle, up_distance=up_distance))
                # throttle too high -> decrease throttle and decrease step size (UP)
                else:
                    up_throttle += up_step
                    log.info(self._clm("FRT", message="Decreasing (UP)", throttle=up_throttle, up_distance=up_distance))

        self.slow_throttle_down = down_throttle
        self.slow_throttle_up = up_throttle
        log.success(self._clm("FRT", slow_throttle_down=self.slow_throttle_down, slow_throttle_up=self.slow_throttle_up))

    async def _find_cps(self):
        """Find counts per second of the motor in both directions"""
        log.info(self._clm("Find CPS", message="Finding cps up and down"))

        # neutral positions must be calibrated and set for present throttles
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions not found")

        if not self.is_home():
            await self.to_home(constants.uncalibrated_home_throttle)

        await self.to(0.1) # move to buffer position 

         # ------------------------------- find cps down ------------------------------ #
        if self.cps_down is None:
            log.info(self._clm("Find CPS", message="Finding cps down"))

            # move to calibration position
            timed_out, time_elapsed = await self.move(
                n_counts=constants.calibration_counts, 
                throttle=constants.ThrottlePresets.SLOW, 
                direction=constants.down, 
                timeout=constants.calibration_timeout
            ) 

            self.cps_down = constants.calibration_counts / time_elapsed # compute cps down
            log.info(self._clm("Find CPS", cps_down=self.cps_down))

        # -------------------------------- find cps up ------------------------------- #
        if self.cps_up is None:
            log.info(self._clm("Find CPS", message="Finding cps up"))

            timed_out, time_elapsed = await self.move(
                n_counts=constants.calibration_counts, 
                throttle=constants.ThrottlePresets.SLOW, 
                direction=constants.up, 
                timeout=constants.calibration_timeout
            )

            if timed_out:
                log.error(self._clm("Find CPS", message="Throttle timed out moving home"))
                return
            
            self.cps_up = constants.calibration_counts / time_elapsed # compute cps up
            log.info(self._clm("Find CPS", cps_up=self.cps_up))

        log.success(self._clm("Find CPS", cps_down=self.cps_down, cps_up=self.cps_up))
        return self.cps_down, self.cps_up

    async def _find_neutrals(self):
        """Find the lower and upper neutral positions of servo motor"""
        log.info(self._clm("Find Neutrals", message="Finding neutral positions"))

        # ensure motor is at home position
        if not self.is_home():
            await self.to_home(constants.uncalibrated_home_throttle)

        step = 0.01 #? should be in constants
        current_throttle = 0.35 #? should be in constants
        
        # continually decrease throttle until both neutral positions are found
        while self.upper_neutral is None or self.lower_neutral is None:
            current_throttle = round(current_throttle - step, 2)
            log.info(self._clm("Find Neutrals", current_throttle=current_throttle))
            timed_out, _ = await self.move(n_counts=2, throttle=current_throttle, timeout=constants.calibration_to_position_timeout, ensure_enabled=True)

            # initial throttle has timed out -> found upper neutral
            if self.upper_neutral is None and timed_out:
                log.info(self._clm("Find Neutrals", message=f"Upper neutral found: {current_throttle}"))
                self.upper_neutral = current_throttle

            # upper neutral found and motor has not timed out -> found lower neutral
            if self.lower_neutral is None and not timed_out and self.upper_neutral is not None:
                log.info(self._clm("Find Neutrals", message=f"Lower neutral found: {current_throttle}"))
                self.lower_neutral = current_throttle + step # add step to account for last iteration

        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions calibrated incorrectly")

        log.info(self._clm("Find Neutrals", lower_neutral=self.lower_neutral, upper_neutral=self.upper_neutral))

    async def calibrate(self, data: SingularCalibrationData):
        """Calibrate the motor to determine lower and upper bounds of motor speed"""
        log.info(self._clm("Calibrate", message="Calibrating Motor"))

        # load calibration data if available
        if data['cps_down'] is not None:
            log.info(self._clm("Calibrate", message="cps down already calibrated"))
            self.cps_down = data['cps_down']
        if data['cps_up'] is not None:
            log.info(self._clm("Calibrate", message="cps up already calibrated"))
            self.cps_up = data['cps_up']
        if data['lower_neutral'] is not None:
            log.info(self._clm("Calibrate", message="lower neutral already calibrated"))
            self.lower_neutral = data['lower_neutral']
        if data['upper_neutral'] is not None:
            log.info(self._clm("Calibrate", message="upper neutral already calibrated"))
            self.upper_neutral = data['upper_neutral']
        if data['slow_throttle_down'] is not None:
            log.info(self._clm("Calibrate", message="slow throttle down already calibrated"))
            self.slow_throttle_down = data['slow_throttle_down']
        if data['slow_throttle_up'] is not None:
            log.info(self._clm("Calibrate", message="slow throttle up already calibrated"))
            self.slow_throttle_up = data['slow_throttle_up']

        self.encoder_feedback_disabled = False # start incrementing encoder counts

        # find neutrals if either is not present
        if self.lower_neutral is None or self.upper_neutral is None:
            await self._find_neutrals()

        # find cps if either is not present
        if self.cps_down is None or self.cps_up is None:
            await self._find_cps()

        await self.to_home() # move back with calibrated neutral positions
        
        self.encoder_feedback_disabled = True

        # ensure calibration was successful
        if self.cps_down is None or self.cps_up is None:
            raise ValueError("CPS incorrectly calibrated")
        if self.lower_neutral is None or self.upper_neutral is None:
            raise ValueError("Neutral positions incorrectly calibrated")

    def is_home(self) -> bool:
        """Check if the motor is at the home position"""
        return self.counts == 0