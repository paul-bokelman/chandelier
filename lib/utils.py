from termcolor import colored
from typing import Optional
import constants

class Logger:
    """Log messages to console with color"""
    def __init__(self):
        pass

    def info(self, msg, override=False):
        if constants.debug or override:
            print(msg)

    def success(self, msg):
        print(colored(msg, 'green'))

    def error(self, msg):
        print(colored(msg, 'red'))

    def warning(self, msg):
        print(colored(msg, 'yellow'))

    def debug(self, msg):
        print(colored(msg, 'blue'))

log = Logger()

def to_pulse(speed: float, direction, up_boost: Optional[float] = 0, down_boost: Optional[float] = 0)->float:
    """Convert motor speed and direction into Servo pulse, speed is a value between 0 and 1"""
    if speed < 0 or speed > 1:
        raise ValueError("Invalid speed value")
    
    # if no boost, set to 0 (no boost), should only be None if no cps data
    if up_boost is None:
        up_boost = 0
    if down_boost is None:
        down_boost = 0
    
    # normalize speed to range
    speed = (constants.max_speed-constants.min_speed) * (speed) + constants.min_speed

    # calculate relative speed with boost depending on direction
    relative_speed = (speed + speed * up_boost) if direction == constants.up else (speed + speed * down_boost)

    log.info(f"Speed: {speed} | Relative Speed: {relative_speed} | Direction: {direction} | Up Boost: {up_boost} | Down Boost: {down_boost}")
    
    if direction == constants.up:
      return -8*(relative_speed-187.5) 
    else:
      return 8*(relative_speed+187.5) 

def calculate_relative_boosts(all_cps: list[Optional[float]]) -> list[Optional[float]]:
    """Calculate relative boosts for all motors based on cps data"""

    # if no cps data, return None for all motors
    if not all_cps:
        return [None] * constants.n_motors

    max_r, min_r = 1, 0 # max and min range
    max_cps, min_cps = max([c for c in all_cps if c]), 0 # max and min cps
    relative_boost = lambda cps: 1 - ((cps - min_cps) * ((max_r - min_r) / (max_cps) - min_cps) + min_r) if cps else None

    return [relative_boost(cps) for cps in all_cps]
    