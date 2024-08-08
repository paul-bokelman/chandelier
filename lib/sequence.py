from typing import Generator
import random as rand
import constants
import math
from lib.mc.motor import Throttle

GeneratedSequence = Generator[tuple[list[float], Throttle], None, None] # return type for generated sequence

class Sequence:
    """Generates sequences for all motors"""
    def __init__(self):
        self.default_throttles = constants.ThrottlePresets.SLOW
        pass

    def random_iteration(self) -> tuple[list[float], Throttle]:
        """Generate a random sequence for all motors"""
        positions = [rand.uniform(0.3, 0.9) for _ in range(constants.n_active_motors)]
        throttles = self.default_throttles
        return (positions, throttles)

    def random(self, iterations: int = 5) -> GeneratedSequence:
        """Generate a random sequence for all motors"""
        for _ in range(iterations):
            yield self.random_iteration()
        
    def wave(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.3, frequency: float = 0.5, step: float = 0.5) -> GeneratedSequence:
        """Generate a wave sequence for all motors"""
        for i in range(iterations):
            positions = [amplitude * math.sin(2 * math.pi * frequency * i + step * j) + translation + amplitude for j in range(constants.n_active_motors)]
            throttles = self.default_throttles
            yield (positions, throttles)

    def alternating(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.4) -> GeneratedSequence:
        """Generate an alternating sequence for all motors"""
        for i in range(iterations):
            positions = [amplitude + translation if (i + j) % 2 == 0 else translation for j in range(constants.n_active_motors)]
            throttles = self.default_throttles
            yield (positions, throttles)