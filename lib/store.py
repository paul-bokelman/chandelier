from typing import Optional, TypedDict
import os
import json
from configuration.config import config
from lib.utils import log
from lib.motor import Motor

class SingularCalibrationData(TypedDict):
  """Schema for singular calibration data"""
  cps_down: Optional[float]
  cps_up: Optional[float]
  lower_neutral: Optional[float]
  upper_neutral: Optional[float]
  throttle_down: Optional[float]
  throttle_up: Optional[float]

class CalibrationData(TypedDict):
  """Schema for calibration data"""
  cps_down: list[Optional[float]]
  cps_up: list[Optional[float]]
  lower_neutral: list[Optional[float]]
  upper_neutral: list[Optional[float]]
  throttle_down: list[Optional[float]]
  throttle_up: list[Optional[float]]

default_data: CalibrationData = {
  "cps_down": [None] * config.get('n_motors'),
  "cps_up": [None] * config.get('n_motors'),
  "lower_neutral": [None] * config.get('n_motors'),
  "upper_neutral": [None] * config.get('n_motors'),
  "throttle_down": [None] * config.get('n_motors'),
  "throttle_up": [None] * config.get('n_motors')
}

class CalibrationStore:
  def __init__(self):
    """Manage persistent calibration data"""
    self.data: CalibrationData = default_data

    # create calibration file if it doesn't exist
    if not os.path.exists(config.get('calibration_file_path')):
      log.info(f"Creating calibration file: {config.get('calibration_file_path')}")
      self.save()

    # read data from file
    with open(config.get('calibration_file_path'), "r") as f:
      data: CalibrationData = json.load(f)
      f.close()
      self.data = data

    valid = self._validate()

    if not valid:
      raise ValueError("Invalid calibration file data")

  def _validate(self) -> bool:
    """Validate current calibration data"""
    if self.data is None:
      log.error("No calibration data found")
      return False
    
    # ensure all keys are present
    for key in self.data.keys():
      if key not in CalibrationData.__annotations__.keys():
        log.error(f"Invalid calibration data: {key}")
        return False
    
    for key in self.data.keys():
      # ensure all channels are present
      if len(self.data[key]) != config.get('n_motors'):
        log.error(f"Invalid calibration data: {key} length: {len(self.data[key])}")
        return False
      
      # ensure all values are floats or None
      for value in self.data[key]:
        if value is not None and not isinstance(value, float):
          log.error(f"Invalid calibration data: {key} value: {value}")
          return False
      
    return True

  def reset(self):
    """Reset calibration data"""
    log.info(f"Resetting calibration data")
    self.data = default_data
    self.save()

  def save(self):
    """Validate and save data to file"""
    log.info(f"Saving calibration data")

    valid = self._validate()

    if not valid:
      log.error("Invalid calibration data")
      raise ValueError("Invalid calibration data")
    
    with open(config.get('calibration_file_path'), "w") as f:
      f.write(json.dumps(self.data))
      f.close()

  def load(self, motors: list[Motor]):
    """Load calibration data from store"""
    for motor in motors:
        data = self.get_entry(motor.channel)
        motor.cps_down = data["cps_down"]
        motor.cps_up = data["cps_up"]
        motor.lower_neutral = data["lower_neutral"]
        motor.upper_neutral = data["upper_neutral"]
        motor.throttle_down = data["throttle_down"]
        motor.throttle_up = data["throttle_up"]
  
  def update_data(self, channel: int, data: SingularCalibrationData):
    """Update calibration data for a specific motor channel"""
    log.info(f"Updating calibration data for channel {channel}")

    # ensure channel is valid
    if channel < 0 or channel >= config.get('n_motors'):
      raise ValueError("Invalid motor channel")

    # update all channel values
    self.data["cps_down"][channel] = data["cps_down"]
    self.data["cps_up"][channel] = data["cps_up"]
    self.data["lower_neutral"][channel] = data["lower_neutral"]
    self.data["upper_neutral"][channel] = data["upper_neutral"]
    self.data["throttle_down"][channel] = data["throttle_down"]
    self.data["throttle_up"][channel] = data["throttle_up"]

    self.save()

  def reset_entry(self, channel: int):
    """Reset calibration data for a specific motor channel"""
    log.info(f"Deleting calibration data by channel")

    # ensure channel is valid
    if channel < 0 or channel >= config.get('n_motors'):
      raise ValueError("Invalid motor channel")

    # reset all channel values
    self.data["cps_down"][channel] = None
    self.data["cps_up"][channel] = None
    self.data["lower_neutral"][channel] = None
    self.data["upper_neutral"][channel] = None
    self.data["throttle_down"][channel] = None
    self.data["throttle_up"][channel] = None

    self.save()
  
  def get_entry(self, channel: int) -> SingularCalibrationData:
    """Get calibration data for a specific motor channel"""
    log.info(f"Getting calibration data for channel {channel}")

    # ensure channel is valid
    if channel < 0 or channel >= config.get('n_motors'):
      raise ValueError("Invalid motor channel")

    # return formatted data
    return {
      "cps_down": self.data["cps_down"][channel],
      "cps_up": self.data["cps_up"][channel],
      "lower_neutral": self.data["lower_neutral"][channel],
      "upper_neutral": self.data["upper_neutral"][channel],
      "throttle_down": self.data["throttle_down"][channel],
      "throttle_up": self.data["throttle_up"][channel]
    }
  
  def get(self) -> CalibrationData:
    """Get all calibration data"""
    return self.data