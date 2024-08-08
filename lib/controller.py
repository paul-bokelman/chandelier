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
    self.active_motors = len(active_motors)
    return active_motors

  def stop_all_motors(self):
    """Stop all motors by calling stop_motor for each motor"""
    for motor in self.motors:
      motor.stop()

  async def move_all_home(self):
    """Move all motors to home position"""
    await asyncio.gather(*[motor.to_home() for motor in self.motors if not motor.disabled])

  async def calibrate(self, reset = False):
    """Find cps down and up for each motor"""
    if reset: 
      log.info("Resetting calibration data")
      self.store.reset()

    log.info("Calibrating motors")

    # calibrate each motor individually simultaneously
    await asyncio.gather(*[motor.calibrate(self.store.get_by_channel(motor.channel)) for motor in self.motors if not motor.disabled])
    log.success("Completed individual calibrations")
    max_cps_up = max([motor.cps_up for motor in self.motors if motor.cps_up is not None]) # get max cps up
    max_cps_down = max([motor.cps_down for motor in self.motors if motor.cps_down is not None]) # get max cps down

    log.info(f"Max CPS Up: {max_cps_up} | Max CPS Down: {max_cps_down}")

    # calculate all relative throttles 
    await asyncio.gather(*[motor.find_relative_throttles(max_cps_up, max_cps_down) for motor in self.motors if not motor.disabled])

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

  async def move_all(self, positions: Union[float, list[float]], throttles: Union[Throttle, list[Throttle]] = constants.ThrottlePresets.SLOW) -> float:
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
    if len(throttles) != len([motor for motor in self.motors if not motor.disabled]):
      raise ValueError("Speed list must be the same length as the number of motors")
    if len(positions) != len([motor for motor in self.motors if not motor.disabled]):
      raise ValueError("Position list must be the same length as the number of motors")
    if len(throttles) != len(positions):
      raise ValueError("Speed and positions must be the same length")
    
    # move each motor to its target position simultaneously
    tasks = await asyncio.gather(*[motor.to(position, throttle) for motor, position, throttle in zip(active_motors, positions, throttles) ])

    # return max elapsed time
    return max([task[1] for task in tasks])
  
  def destroy(self):
    """Destroy motor controller by stopping all motors"""
    self.stop_all_motors()
    self.kit._pca.deinit()