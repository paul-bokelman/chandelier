from termcolor import colored
from typing import Optional
import constants

class Logger:
    def __init__(self):
        pass

    def info(self, msg):
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

def to_pulse(speed: float, direction, up_boost: float, down_boost: float)->float:
    """Convert motor speed and direction into Servo pulse, speed is a value between 0 and 1"""
    if speed < 0 or speed > 1:
        raise ValueError("Invalid speed value")
    
    # normalize speed to range
    speed = (constants.max_speed-constants.min_speed) * (speed) + constants.min_speed
    
    # calculate relative speed with boost depending on direction
    relative_speed = (speed + speed * up_boost) if direction == constants.up else (speed - speed * down_boost)
    
    if direction == constants.up:
      return -8*(relative_speed-187.5) 
    else:
      return 8*(relative_speed+187.5) 
    

def calculate_relative_boosts(all_cps: list[Optional[float]]) -> list[Optional[float]]:
    """Calculate and scale relative speeds"""

    # if no cps data, return None for all motors
    if not all_cps:
        return [None] * constants.n_motors

    max_r, min_r = 1, 0 # max and min range
    max_cps, min_cps = max([c for c in all_cps if c]), 0 # max and min cps
    relative_speed = lambda cps: 1 - ((cps -  min_cps) * ((max_r - min_r) / (max_cps) - min_cps) + min_r) if cps else None

    return [relative_speed(cps) for cps in all_cps]
    