from termcolor import colored

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

def to_pulse(speed: float)->float:
    """Convert motor speed and direction into Servo pulse, speed is a value between -1 and 1"""
    if speed < -1 or speed > 1:
        raise ValueError("Invalid speed value") 

    p = lambda x: 8 * ((abs(x) * 100)+187.5) # convert speed to pulse

    if speed < 0: 
        return -p(speed) # reverse direction
    else:
        return p(speed) # forward direction