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

class CalibrationData(TypedDict):
  """Schema for calibration data"""
  cps_down: list[Optional[float]]
  cps_up: list[Optional[float]]
  lower_neutral: list[Optional[float]]
  upper_neutral: list[Optional[float]]

default_calibration_data: CalibrationData = {
  "cps_down": [None] * constants.n_motors,
  "cps_up": [None] * constants.n_motors,
  "lower_neutral": [None] * constants.n_motors,
  "upper_neutral": [None] * constants.n_motors
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
  
  def get(self, mode: DataMode) -> list[Optional[float]]:
    """Get specific calibration data from file"""
    log.info(f"Getting calibration data from {mode.name}")
    
    assert mode in DataMode, "Invalid calibration mode"
    key = str(mode.name.lower())
    assert key in self.data, "Invalid calibration key"
    
    return self.data[key]
  
  def get_by_channel(self, channel: int) -> list[Optional[float]]:
    """Get calibration data for a specific motor channel"""
    log.info(f"Getting calibration data by channel")
    if channel < 0 or channel >= constants.n_motors:
      raise ValueError("Invalid motor channel")

    return [
      self.data["cps_down"][channel],
      self.data["cps_up"][channel],
      self.data["lower_neutral"][channel],
      self.data["upper_neutral"][channel]
    ]
  
  def load(self) -> CalibrationData:
    """Load calibration data from file"""
    log.info(f"Loading calibration data")
    return self.data