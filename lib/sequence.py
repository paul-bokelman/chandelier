from typing import Generator, Optional
import random as rand
import math
from configuration.config import config
from lib.utils import log

# return type for generated sequence
IteratorReturnValue = tuple[list[float], Optional[float]]
GeneratedSequence = Generator[IteratorReturnValue, None, None]

class Sequence:
    """Generates sequences for all motors"""
    def __init__(self) -> None:
        self.default_throttles = [None] * config.get('n_motors')

    def random_iteration(self) -> IteratorReturnValue:
        """Generate a random sequence for all motors"""
        min, max = config.get('random_params')
        positions = [rand.uniform(min, max) for _ in range(config.get('n_motors'))]
        throttles = self.default_throttles
        return (positions, throttles)

    def random(self, iterations: int = 5) -> GeneratedSequence:
        """Generate a random sequence for all motors"""
        for _ in range(iterations):
            yield self.random_iteration()

    def wave_iteration(self, i: int) -> IteratorReturnValue:
        """Generate a wave sequence for all motors"""
        amplitude, translation, frequency, step = config.get('wave_params')
        positions = [amplitude * math.sin(math.pi * frequency * i + step * j) + translation + amplitude for j in range(config.get('n_motors'))]
        throttles = self.default_throttles
        log.info(f"Wave positions: {positions}, Params: {amplitude, frequency, i, step}")

        return (positions, throttles)
        
    def wave(self, iterations: int = 5) -> GeneratedSequence:
        """Generate a wave sequence for all motors"""
        for i in range(iterations):
            yield self.wave_iteration(i)

    def alternating_iteration(self, i: int) -> IteratorReturnValue:
        """Generate an alternating sequence for all motors"""
        amplitude, translation = config.get('alternating_params')
        positions = [amplitude + translation if (i + j) % 2 == 0 else translation for j in range(config.get('n_motors'))]
        log.info(f"Alternating positions: {positions}, Params: {amplitude, translation}")
        throttles = self.default_throttles
        return (positions, throttles)

    def alternating(self, iterations: int = 5) -> GeneratedSequence:
        """Generate an alternating sequence for all motors"""
        for i in range(iterations):
            yield self.alternating_iteration(i)