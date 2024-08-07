from termcolor import colored
import constants

class Logger:
    """Log messages to console with color"""
    def __init__(self):
        pass

    def info(self, msg, override=False):
        if constants.debug or override:
            print(msg)

    def success(self, msg, override=False):
        if constants.debug or override:
            print(colored(msg, 'green'))

    def error(self, msg):
        print(colored(msg, 'red'))

    def warning(self, msg):
        print(colored(msg, 'yellow'))

    def debug(self, msg):
        print(colored(msg, 'blue'))

log = Logger()

def to_seconds(elapsed_time: float) -> int:
    """Return seconds elapsed since start time"""
    return int(elapsed_time)

