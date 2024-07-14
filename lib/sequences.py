from enum import Enum
import random
import constants
from lib.mc.controller import MotorController

class Sequence:
    def __init__(self, mc: MotorController):
        self.mc = mc

    def start(self):
        pass

    def stop(self):
        self.mc.stop_all_motors()

    def transition(self, to):
        pass
    
    def reset(self):
        pass

class RandomSequence(Sequence):
    def __init__(self, mc: MotorController):
        super().__init__(mc)
        self.mc = MotorController()

    async def start(self, iterations: int = 5):
        await self.mc.move_all_home()

        # run random sequence until stopped
        for _ in range(iterations):
            positions = [random.uniform(0.5, 1) for _ in range(constants.n_motors)]
            speeds = [random.uniform(0.4, 0.7) for _ in range(constants.n_motors)]
            await self.mc.move_all(positions, speeds)

class AlternatingSequence(Sequence):
    """Alternate between two positions and speeds for all motors"""
    def __init__(self, mc: MotorController):
        super().__init__(mc)
        self.mc = MotorController()

    async def start(self, iterations: int = 5, amplitude: float = 0.5, translation: float = 0.5, speed: float = 0.5):
        await self.mc.move_all_home()

        # run alternating sequence until stopped
        for i in range(iterations):
            positions = [0.5, 0.5, 0.5, 0.5]
            if i % 2 == 0:
                await self.mc.move_all(positions, speeds)

            positions = [0.5, 0.5, 0.5, 0.5]
            speeds = [speed] * constants.n_motors
            await self.mc.move_all(positions, speeds)

        

# class Sequences(Enum):
#     random = RandomSequence
#     wave = 1