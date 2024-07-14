from typing import Generator
import random as rand
import constants
import math

GeneratedSequence = Generator[tuple[list[float], list[float]], None, None] # return type for generated sequence

class Sequence:
    def __init__(self):
        pass

    def random(self, iterations: int = 5) -> GeneratedSequence:
        """Generate a random sequence for all motors"""
        for _ in range(iterations):
            positions = [rand.uniform(0.5, 1) for _ in range(constants.n_motors)]
            speeds = [rand.uniform(0.4, 0.7) for _ in range(constants.n_motors)]
            yield (positions, speeds)
        
    def wave(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.3, frequency: float = 0.5, step: float = 0.5) -> GeneratedSequence:
        """Generate a wave sequence for all motors"""
        for i in range(iterations):
            positions = [amplitude * math.sin(2 * math.pi * frequency * i + step * j) + translation for j in range(constants.n_motors)]
            speeds = [rand.uniform(0.4, 0.7) for _ in range(constants.n_motors)]
            yield (positions, speeds)

    def alternating(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.3) -> GeneratedSequence:
        """Generate an alternating sequence for all motors"""
        for i in range(iterations):
            positions = [amplitude + translation if (i + j) % 2 == 0 else translation for j in range(constants.n_motors)]
            speeds = [rand.uniform(0.4, 0.7) for _ in range(constants.n_motors)]
            yield (positions, speeds)


    