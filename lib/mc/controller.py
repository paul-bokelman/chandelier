from typing import Union
import asyncio
import constants
from adafruit_servokit import ServoKit
from lib.store import Store, CalibrationData
from lib.mc.motor import Motor
from lib.utils import calculate_relative_boosts, log

class MotorController:
  """Main motor controller class for calibrating, saving, and manipulating servos"""
  def __init__(self, debug = False) -> None:
    self.debug = debug
    self.store = Store()
    self.kit = ServoKit(channels=16)
    self.motors = [Motor(i,self.kit.continuous_servo[i]) for i in range(constants.n_motors)]

  def stop_all_motors(self):
    """Stop all motors by calling stop_motor for each motor"""
    for motor in self.motors:
      motor.stop()

  async def move_all_home(self):
    """Move all motors to home position"""
    await asyncio.gather(*[motor.to_home() for motor in self.motors])

  async def calibrate(self, reset = False):
    """Find cps down and up for each motor"""
    if reset: self.store.reset()

    # calibrate each motor simultaneously
    await asyncio.gather(*[motor.calibrate(self.store.get_by_channel(motor.channel)) for motor in self.motors])

    # calculate and assign relative speeds
    # up_boosts = calculate_relative_boosts([motor.cps_up for motor in self.motors])
    # down_boosts = calculate_relative_boosts([motor.cps_down for motor in self.motors])

    # for (motor, up_boost, down_boost) in zip(self.motors, up_boosts, down_boosts):
    #   motor.up_boost = up_boost
    #   motor.down_boost = down_boost

  async def move_all(self, positions: Union[float, list[float]], speeds: Union[float, list[float]] = 0.2) -> int:
    """Move all motors to specific positions. Positions is a list of floats from 0 to 1 representing the position of each motor (0 is home, 1 is max). Returns total elapsed time since start."""

    # convert single position to list of speeds
    if not isinstance(speeds, list):
      assert isinstance(speeds, float), "Speed must be a float"
      assert 0 <= speeds <= 1, "Speed must be between 0 and 1"
      speeds = [speeds] * constants.n_motors

    if not isinstance(positions, list):
      assert isinstance(positions, float), "Positions must be a float"
      assert 0 <= positions <= 1, "Position must be between 0 and 1"
      positions = [positions] * constants.n_motors

    assert isinstance(speeds, list), "Speed must be a list of floats"
    assert isinstance(positions, list), "Positions must be a list of floats"

    if not all(0 <= s <= 1 for s in speeds):
      raise ValueError("Speeds must be between 0 and 1")
    
    if not all(0 <= p <= 1 for p in positions):
      raise ValueError("Positions must be between 0 and 1")
    
    if len(speeds) != constants.n_motors:
      raise ValueError("Speed list must be the same length as the number of motors")
    
    if len(positions) != constants.n_motors:
      raise ValueError("Positions list must be the same length as the number of motors")
    
    # todo: speeds unused

    # move each motor to its target position simultaneously
    tasks = await asyncio.gather(*[motor.to(position) for motor, position, speed in zip(self.motors, positions, speeds)])

    return max([task[1] for task in tasks]) # return max elapsed time

  def save_calibration(self):
    """Save calibration data with store"""
    log.info("Saving calibration data")
    data = CalibrationData(
      cps_down=[motor.cps_down for motor in self.motors],
      cps_up=[motor.cps_up for motor in self.motors],
      lower_neutral=[motor.lower_neutral for motor in self.motors],
      upper_neutral=[motor.upper_neutral for motor in self.motors]
    )
    self.store.save(data)
    log.success("Calibration data saved")