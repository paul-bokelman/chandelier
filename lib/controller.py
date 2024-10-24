from typing import Union, Optional
import asyncio
from adafruit_servokit import ServoKit
from configuration.config import config
from lib.store import CalibrationStore, SingularCalibrationData
from lib.motor import Motor
from lib.utils import log

class MotorController:
  """Main motor controller class for calibrating, saving, and manipulating servos"""
  def __init__(self, debug = False) -> None:
    self.debug = debug
    self.store = CalibrationStore()
    self.kit = ServoKit(channels=16)
    self.motors: list[Motor] = [Motor(i,self.kit.continuous_servo[i]) for i in range(config.get('n_motors'))]

  def stop_all_motors(self):
    """Stop all motors by calling stop_motor for each motor"""
    for motor in self.motors:
      motor.stop()

  async def move_all_home(self, throttle: Optional[float] = None):
    """
    Move all motors to home position
    
    Parameters:
      throttle (Optional[float]): Throttle value for all motors
    """
    await asyncio.gather(*[motor.to_home(throttle) for motor in self.motors])

  async def move_all(self, positions: Union[float, list[float]], throttles: Union[None, float, list[float], list[None]] = None):
    """Move all motors to specific positions with specific throttles"""

    # singular value -> convert to list of that value
    if isinstance(throttles, float):
      if not (0 <= throttles <= 1):
        raise ValueError(f"Position must be between 0 and 1, received {throttles}")
      throttles = [throttles] * len(self.motors)

    # None -> convert to list of None
    if throttles is None:
      throttles = [None] * len(self.motors)

    # singular value -> convert to list of that value
    if isinstance(positions, float):
      if not (0 <= positions <= 1):
        raise ValueError(f"Position must be between 0 and 1, received {positions}")
      positions = [positions] * len(self.motors)

    # ensure both are lists
    if not isinstance(throttles, list):
      raise ValueError("Throttles must be a list")
    
    if not isinstance(positions, list):
      raise ValueError("Positions must be a list")

    # ensure lengths are the same
    if len(throttles) != len(self.motors):
      raise ValueError("Throttles must be the same length as the number of motors")
    if len(positions) != len(self.motors):
      raise ValueError("Positions must be the same length as the number of motors")

    # move each motor to its target position simultaneously
    await asyncio.gather(*[motor.to(position, throttle) for motor, position, throttle in zip(self.motors, positions, throttles)])

  async def calibrate_home_positions(self):
    """Calibrate home positions for all motors"""
    log.info("Calibrating home positions...")

    # calibrate home positions for all enabled motors
    await asyncio.gather(*[motor.calibrate_home() for motor in self.motors if not motor.disabled and not motor.dead])

    log.success("Calibrated home positions")

  def load_calibration_data(self):
    """Load calibration data from store. This method is used when store is externally validated."""
    self.store.load(self.motors)

  async def calibrate(self, reset: Union[bool, list[int]] = False):
    """
    Calibrate all motors, optionally resetting calibration data
    
    Parameters:
      reset (Union[bool, list[int]]): Reset calibration data for all motors or specific motors
    """
    
    # reset all calibration data
    if isinstance(reset, bool) and reset: 
      self.store.reset()

    # reset calibration data for specific motors
    if isinstance(reset, list):
      for channel in reset:
        self.store.reset_entry(channel)

    self.store.load(self.motors) # load calibration data into motors

    log.info("Calibrating motors...")

    # independently calibrate each motor (skips if already calibrated)
    await asyncio.gather(*[motor.calibrate_independent() for motor in self.motors])

    log.success("Completed independent calibrations")
    
    # find max cps up and down from group -> used to calibrate relative throttles
    max_cps_up = max([motor.cps_up for motor in self.motors if motor.cps_up is not None]) # get max cps up
    max_cps_down = max([motor.cps_down for motor in self.motors if motor.cps_down is not None]) # get max cps down

    log.info("Calibrating relative throttles")

    # calibrate all relative throttles 
    await asyncio.gather(*[motor.calibrate_relative_throttles(max_cps_up, max_cps_down) for motor in self.motors])

    log.info("Calibrated relative throttles")
    log.info("Saving calibration data...")

    # update all calibration data in store
    for motor in self.motors:
      data: SingularCalibrationData = {
        "cps_down": motor.cps_down,
        "cps_up": motor.cps_up,
        "lower_neutral": motor.lower_neutral,
        "upper_neutral": motor.upper_neutral,
        "throttle_down": motor.throttle_down,
        "throttle_up": motor.throttle_up
      }

      self.store.update_data(motor.channel, data)

    log.info("Calibration data saved")
    log.success("Calibration complete")

  async def recover_all(self):
    """Attempt to recover all disabled motors"""
    await asyncio.gather(*[motor.recover() for motor in self.motors if motor.disabled])
