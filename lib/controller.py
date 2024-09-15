from typing import Union
from lib.motor import Throttle
import asyncio
import constants
from adafruit_servokit import ServoKit
from lib.store import Store, CalibrationData
from lib.motor import Motor
from lib.utils import log

class MotorController:
  """Main motor controller class for calibrating, saving, and manipulating servos"""
  def __init__(self, debug = False) -> None:
    self.debug = debug
    self.store = Store()
    self.kit = ServoKit(channels=16)
    self.motors = [Motor(i,self.kit.continuous_servo[i]) for i in range(constants.n_motors)]
    self.n_active_motors = constants.n_motors - len(constants.initial_disabled_motors)

  def _get_active_motors(self):
    """Compute active motors based on disabled motors"""
    active_motors = [motor for motor in self.motors if not motor.disabled]
    self.n_active_motors = len(active_motors)
    return active_motors

  def stop_all_motors(self):
    """Stop all motors by calling stop_motor for each motor"""
    for motor in self.motors:
      motor.stop()

  async def move_all_home(self, throttle: Throttle = constants.ThrottlePresets.SLOW):
    """Move all motors to home position"""
    active_motors = self._get_active_motors()
    await asyncio.gather(*[motor.to_home(throttle) for motor in active_motors])

  async def calibrate(self, reset = False, load_values = False, update: list[int] = []):
    """Find cps down and up for each motor"""
    if reset: 
      log.info("Resetting calibration data")
      self.store.reset()

    # all valid data is present -> load and exit
    if load_values:
      self.load_calibration_data()
      log.success("Calibration data loaded")
      return

    log.info("Calibrating motors")
    self.load_calibration_data() # load calibration data if available

    active_motors = self._get_active_motors()
    motors = active_motors if len(update) <= 0 else [motor for motor in self.motors if motor.channel in update]
    prior_max_cps_up = max([motor.cps_up for motor in active_motors if motor.cps_up is not None]) # get max cps up
    prior_max_cps_down = max([motor.cps_down for motor in active_motors if motor.cps_down is not None]) # get max cps down

    # update specific motors if valid configuration provided
    if len(update) > 0 and self.calibration_is_valid():
      # remove current calibration data for motors to be updated
      for motor in motors:
        self.store.delete(channel=motor.channel)

    # recalibrate required motors
    await asyncio.gather(*[motor.calibrate() for motor in motors])

    log.success("Completed individual calibrations")
    max_cps_up = max([motor.cps_up for motor in self.motors if motor.cps_up is not None]) # get max cps up
    max_cps_down = max([motor.cps_down for motor in self.motors if motor.cps_down is not None]) # get max cps down

    log.info(f"Max CPS Up: {max_cps_up} | Max CPS Down: {max_cps_down}")
    log.info("Calculating relative throttles")

    # calculate all relative throttles 
    if prior_max_cps_up != max_cps_up or prior_max_cps_down != max_cps_down:
      log.warning("Max CPS values have changed. Recalibrating relative throttles")
      await asyncio.gather(*[motor.find_relative_throttles(max_cps_up, max_cps_down) for motor in active_motors])

    log.success("Calibration complete")

    log.info("Saving calibration data")
    data = CalibrationData(
      cps_down=[motor.cps_down for motor in self.motors],
      cps_up=[motor.cps_up for motor in self.motors],
      lower_neutral=[motor.lower_neutral for motor in self.motors],
      upper_neutral=[motor.upper_neutral for motor in self.motors],
      slow_throttle_down=[motor.slow_throttle_down for motor in self.motors],
      slow_throttle_up=[motor.slow_throttle_up for motor in self.motors]
    )

    self.store.save(data)
    log.success("Calibration data saved")

  def load_calibration_data(self):
    """Load calibration data from store"""
    for motor in self.motors:
        data = self.store.get_by_channel(motor.channel)
        motor.cps_down = data["cps_down"]
        motor.cps_up = data["cps_up"]
        motor.lower_neutral = data["lower_neutral"]
        motor.upper_neutral = data["upper_neutral"]
        motor.slow_throttle_down = data["slow_throttle_down"]
        motor.slow_throttle_up = data["slow_throttle_up"]

  def calibration_is_valid(self):
    """Check if calibration data is valid"""
    data = self.store.load()

    if data is None:
      log.error("No calibration data found")
      return False
    
    for key in data.keys():
      # ensure all keys are present
      if len(data[key]) != constants.n_motors:
        log.error(f"Invalid calibration data: {key} length: {len(data[key])}")
        return False
      # ensure all values are floats
      if not all(isinstance(i, float) for i in data[key]):
        log.error(f"Invalid calibration data: {key} values")
        return False
      
    return True

  async def move_all(self, positions: Union[float, list[float]], throttles: Union[Throttle, list[Throttle]] = constants.ThrottlePresets.SLOW):
    """Move all motors to specific positions. Positions is a list of floats from 0 to 1 representing the position of each motor (0 is home, 1 is max). Returns total elapsed time since start."""

    active_motors = self._get_active_motors()

    # singular value -> convert to list of that value
    if isinstance(throttles, Throttle):
      if isinstance(throttles, float):
        if not (0 <= throttles <= 1):
          raise ValueError(f"Throttle must be between 0 and 1, received {throttles}")
      throttles = [throttles] * self.n_active_motors

    # singular value -> convert to list of that value
    if isinstance(positions, float):
      if not (0 <= positions <= 1):
        raise ValueError(f"Position must be between 0 and 1, received {positions}")
      positions = [positions] * self.n_active_motors

    # ensure both are lists
    if not isinstance(throttles, list):
      raise ValueError("Speed must be a list of Throttle")
    
    if not isinstance(positions, list):
      raise ValueError("Positions must be a list of floats")

    # ensure all floats are between 0 and 1
    if not all(0 <= s <= 1 for s in throttles if isinstance(s, float)):
      raise ValueError(f"Throttles must be between 0 and 1 if float, received {throttles}")
    if not all(0 <= p <= 1 for p in positions):
      raise ValueError(f"Positions must be between 0 and 1, received {positions}")  

    # ensure lengths are the same
    if len(throttles) < self.n_active_motors:
      raise ValueError("Throttle list must be the same length as or longer than the number of motors")
    if len(positions) < self.n_active_motors:
      raise ValueError("Position list must be the same length as or longer than the number of motors")
    if len(throttles) != len(positions):
      log.error(f"LEN MISMATCH: Throttles: {throttles}, Positions: {positions}")
      # raise ValueError("Speed and positions must be the same length")
    
    # move each motor to its target position simultaneously
    await asyncio.gather(*[motor.to(position, throttle) for motor, position, throttle in zip(active_motors, positions, throttles) ])
  
  def destroy(self):
    """Destroy motor controller by stopping all motors"""
    self.stop_all_motors()
    self.kit._pca.deinit()