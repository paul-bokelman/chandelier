from typing import Optional, TypedDict
from enum import Enum
import os
import json
import constants
from lib.utils import log

class DataMode(Enum):
  """Enum for calibration modes"""
  counts = 0
  cps_down = 1
  cps_up = 2
  min_down_speed = 3
  min_up_speed = 4

class CalibrationData(TypedDict):
  """Schema for calibration data"""
  counts: list[Optional[float]] # currently unused
  cps_down: list[Optional[float]]
  cps_up: list[Optional[float]]
  min_down_speed: list[Optional[float]]
  min_up_speed: list[Optional[float]]

default_calibration_data: CalibrationData = {
  "counts": [None] * constants.n_motors,
  "cps_down": [None] * constants.n_motors,
  "cps_up": [None] * constants.n_motors,
  "min_down_speed": [None] * constants.n_motors,
  "min_up_speed": [None] * constants.n_motors
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
    """Get calibration data from file"""
    log.info(f"Getting calibration data from {mode.name}")
    if mode not in DataMode:
      raise ValueError("Invalid calibration mode")
    
    return self.data[mode.name]
  
  def get_by_pin(self, pin: int) -> list[Optional[float]]:
    """Get calibration data for a specific motor pin"""
    log.info(f"Getting calibration data by pin")
    if pin < 0 or pin >= constants.n_motors:
      raise ValueError("Invalid motor pin")

    return [
      self.data["counts"][pin],
      self.data["cps_down"][pin],
      self.data["cps_up"][pin],
      self.data["min_down_speed"][pin],
      self.data["min_up_speed"][pin]
    ]
  
  def load(self) -> CalibrationData:
    """Load calibration data from file"""
    log.info(f"Loading calibration data")
    return self.data