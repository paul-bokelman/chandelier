from PCA9685 import pwm
from lib.utils import to_pulse
import constants

class Motor:
    def __init__(self, id) -> None:
        self.id = id

    def set(self, speed: float):
        """
        Set a specific motor to a specific speed
        """
        pwm.setServoPulse(self.id, to_pulse(speed))

    def stop(self):
        pwm.setServoPulse(self.id, constants.stop_pulse) 

