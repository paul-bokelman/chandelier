from termcolor import colored
import constants

def info(msg):
    print(colored(msg, 'grey'))

def success(msg):
    print(colored(msg, 'green'))

def error(msg):
    print(colored(msg, 'red'))

def warning(msg):
    print(colored(msg, 'yellow'))

def debug(msg):
    print(colored(msg, 'blue'))

def to_pulse(percent_speed: float)->float:
    """
    Convert motor speed and direction into Servo pulse
    """
    if constants.Up_Dir_CCW:
      return -8*(percent_speed-187.5) 
    else:
      return 8*(percent_speed+187.5) 