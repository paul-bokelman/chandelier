from typing import Optional, TypedDict
from enum import Enum
import os
import json
import constants
from lib.utils import log

class DataMode(Enum):
  """Enum for calibration modes"""
  CPS_DOWN = 0
  CPS_UP = 1
  LOWER_NEUTRAL = 2
  UPPER_NEUTRAL = 3
  SLOW_THROTTLE_DOWN = 4
  SLOW_THROTTLE_UP = 5

class SingularCalibrationData(TypedDict):
  """Schema for singular calibration data"""
  cps_down: Optional[float]
  cps_up: Optional[float]
  lower_neutral: Optional[float]
  upper_neutral: Optional[float]
  slow_throttle_down: Optional[float]
  slow_throttle_up: Optional[float]

class CalibrationData(TypedDict):
  """Schema for calibration data"""
  cps_down: list[Optional[float]]
  cps_up: list[Optional[float]]
  lower_neutral: list[Optional[float]]
  upper_neutral: list[Optional[float]]
  slow_throttle_down: list[Optional[float]]
  slow_throttle_up: list[Optional[float]]

default_calibration_data: CalibrationData = {
  "cps_down": [None] * constants.n_motors,
  "cps_up": [None] * constants.n_motors,
  "lower_neutral": [None] * constants.n_motors,
  "upper_neutral": [None] * constants.n_motors,
  "slow_throttle_down": [None] * constants.n_motors,
  "slow_throttle_up": [None] * constants.n_motors
}

class Store:
  """Store and read calibration data"""

  def __init__(self) -> None:
    """Read or create calibration file"""
    # create calibration file if it doesn't exist
    if not os.path.exists(constants.calibrations_file_path):
      log.info(f"Creating calibration file: {constants.calibrations_file_path}")
      self.save(default_calibration_data)

    # read data from file
    with open(constants.calibrations_file_path, "r") as f:
      data = json.load(f)
      f.close()
      self.data = data

  def reset(self):
    """Reset calibration data"""
    log.info(f"Resetting calibration data")
    self.save(default_calibration_data)

  def save(self, data: CalibrationData) -> None:
    """Save calibration data to file"""
    log.info(f"Saving calibration data")
    
    with open(constants.calibrations_file_path, "w") as f:
      f.write(json.dumps(data))
      f.close()

    self.data = data

  def delete(self, channel: int, ) -> None:
    """Delete calibration data for a specific motor channel"""
    log.info(f"Deleting calibration data by channel")
    if channel < 0 or channel >= constants.n_motors:
      raise ValueError("Invalid motor channel")

    self.data["cps_down"][channel] = None
    self.data["cps_up"][channel] = None
    self.data["lower_neutral"][channel] = None
    self.data["upper_neutral"][channel] = None
    self.data["slow_throttle_down"][channel] = None
    self.data["slow_throttle_up"][channel] = None

    self.save(self.data)
  
  def get_by_channel(self, channel: int) -> SingularCalibrationData:
    """Get calibration data for a specific motor channel"""
    log.info(f"Getting calibration data for channel {channel}")
    if channel < 0 or channel >= constants.n_motors:
      raise ValueError("Invalid motor channel")

    return {
      "cps_down": self.data["cps_down"][channel],
      "cps_up": self.data["cps_up"][channel],
      "lower_neutral": self.data["lower_neutral"][channel],
      "upper_neutral": self.data["upper_neutral"][channel],
      "slow_throttle_down": self.data["slow_throttle_down"][channel],
      "slow_throttle_up": self.data["slow_throttle_up"][channel]
    }
  
  def load(self) -> CalibrationData:
    """Load calibration data from file"""
    log.info(f"Loading calibration data")
    return self.data

  