from typing import Generator
import random as rand
import constants
import math
from lib.mc.motor import Throttle

GeneratedSequence = Generator[tuple[list[float], Throttle], None, None] # return type for generated sequence

class Sequence:
    def __init__(self):
        self.default_throttles = constants.ThrottlePresets.SLOW
        pass

    def random(self, iterations: int = 5) -> GeneratedSequence:
        """Generate a random sequence for all motors"""
        for _ in range(iterations):
            positions = [rand.uniform(0.5, 1) for _ in range(constants.n_motors)]
            throttles = self.default_throttles
            yield (positions, throttles)
        
    def wave(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.3, frequency: float = 0.5, step: float = 0.5) -> GeneratedSequence:
        """Generate a wave sequence for all motors"""
        for i in range(iterations):
            positions = [amplitude * math.sin(2 * math.pi * frequency * i + step * j) + translation + amplitude for j in range(constants.n_motors)]
            throttles = self.default_throttles
            yield (positions, throttles)

    def alternating(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.4) -> GeneratedSequence:
        """Generate an alternating sequence for all motors"""
        for i in range(iterations):
            positions = [amplitude + translation if (i + j) % 2 == 0 else translation for j in range(constants.n_motors)]
            throttles = self.default_throttles
            yield (positions, throttles)


    