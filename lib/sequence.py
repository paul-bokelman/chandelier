from typing import Generator
import random as rand
import constants
import math
from lib.motor import Throttle
from lib.utils import log

GeneratedSequence = Generator[tuple[list[float], Throttle], None, None] # return type for generated sequence

class Sequence:
    """Generates sequences for all motors"""
    def __init__(self, n_active_motors: int = constants.n_motors) -> None:
        self.default_throttles = constants.ThrottlePresets.SLOW
        self.n_active_motors = n_active_motors

    def random_iteration(self) -> tuple[list[float], Throttle]:
        """Generate a random sequence for all motors"""
        positions = [rand.uniform(0.3, 0.7) for _ in range(self.n_active_motors)]
        throttles = self.default_throttles
        return (positions, throttles)

    def random(self, iterations: int = 5) -> GeneratedSequence:
        """Generate a random sequence for all motors"""
        for _ in range(iterations):
            yield self.random_iteration()

    def wave_iteration(self, i: int, amplitude: float = 0.1, translation: float = 0.2, frequency: float = 0.5, step: float = 0.5) -> tuple[list[float], Throttle]:
        """Generate a wave sequence for all motors"""
        positions = [amplitude * math.sin(math.pi * frequency * i + step * j) + translation + amplitude for j in range(self.n_active_motors)]
        throttles = self.default_throttles
        log.info(f"Wave positions: {positions}, Params: {amplitude, frequency, i, step}")

        print(positions, [amplitude, frequency, i, step])
        return (positions, throttles)
        
    def wave(self, iterations: int = 5, amplitude: float = 0.1, translation: float = 0.2, frequency: float = 0.5, step: float = 0.5) -> GeneratedSequence:
        """Generate a wave sequence for all motors"""
        for i in range(iterations):
            yield self.wave_iteration(i, amplitude, translation, frequency, step)

    def alternating_iteration(self, i: int, amplitude: float = 0.4, translation: float = 0.3) -> tuple[list[float], Throttle]:
        """Generate an alternating sequence for all motors"""
        positions = [amplitude + translation if (i + j) % 2 == 0 else translation for j in range(self.n_active_motors)]
        log.info(f"Alternating positions: {positions}, Params: {amplitude, translation}")
        throttles = self.default_throttles
        return (positions, throttles)

    def alternating(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.4) -> GeneratedSequence:
        """Generate an alternating sequence for all motors"""
        for i in range(iterations):
            yield self.alternating_iteration(i, amplitude, translation)