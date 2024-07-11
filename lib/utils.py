from termcolor import colored
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

def to_pulse(speed: float, direction)->float:
    """Convert motor speed and direction into Servo pulse, speed is a value between 0 and 1"""
    if speed < 0 or speed > 1:
        raise ValueError("Invalid speed value") 
    
    if direction == constants.up:
      return -8*((speed * 100)-187.5) 
    else:
      return 8*((speed * 100)+187.5) 