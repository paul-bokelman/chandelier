from termcolor import colored
import constants

class Logger:
    def __init__(self):
        pass

    def info(self, msg):
        print(colored(msg, 'grey'))

    def success(self, msg):
        print(colored(msg, 'green'))

    def error(self, msg):
        print(colored(msg, 'red'))

    def warning(self, msg):
        print(colored(msg, 'yellow'))

    def debug(self, msg):
        print(colored(msg, 'blue'))

log = Logger()

def to_pulse(percent_speed: float)->float:
    """
    Convert motor speed and direction into Servo pulse
    """
    return 8*(percent_speed+187.5) 