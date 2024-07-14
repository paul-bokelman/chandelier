from enum import Enum
from typing import Generator
import random as rand
import constants
from lib.mc.controller import MotorController

class Sequence:
    def __init__(self):
        pass

    def random(self, iterations: int = 5) -> Generator[tuple[list[float], list[float]], None, None]:
        for _ in range(iterations):
            positions = [rand.uniform(0.5, 1) for _ in range(constants.n_motors)]
            speeds = [rand.uniform(0.4, 0.7) for _ in range(constants.n_motors)]
            yield (positions, speeds)
        
    def wave(self, iterations: int = 5) -> Generator[tuple[list[float], list[float]]]:
        yield ([], [])

    def alternating(self, iterations: int = 5, amplitude: float = 0.4, translation: float = 0.3) -> Generator[tuple[list[float], list[float]], None, None]:
        for i in range(iterations):
            positions = [amplitude + translation if (i + j) % 2 == 0 else translation for j in range(constants.n_motors)]
            speeds = [rand.uniform(0.4, 0.7) for _ in range(constants.n_motors)]
            yield (positions, speeds)


    