from typing import Optional
from PCA9685 import pwm
from lib.utils import to_pulse
import constants

class Motor:
    def __init__(self, id: int, is_home: bool = False) -> None:
        self.id = id
        self.home = is_home
        self.last_count_time = 0
        self.encoder_count = 0
        self.encoder_pin = constants.encoder_pins[id]
        self.prev_encoder_reading: Optional[int] = None

    def set(self, speed: float):
        """
        Set a specific motor to a specific speed
        """
        pwm.setServoPulse(self.id, to_pulse(speed))

    def stop(self):
        pwm.setServoPulse(self.id, constants.stop_pulse) 

    def is_home(self):
        """
        Returns whether the motor is at home or not
        """
        return self.home