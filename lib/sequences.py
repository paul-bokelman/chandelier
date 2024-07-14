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
    
    def reset(self):
        pass

class RandomSequence(Sequence):
    def __init__(self, mc: MotorController):
        super().__init__(mc)
        self.mc = MotorController()
        

    async def start(self):
        await self.mc.move_all_home()

        # run random sequence until stopped
        while True:
            positions = [random.uniform(0.5, 1) for _ in range(constants.n_motors)]
            speeds = [random.uniform(0.2, 0.7) for _ in range(constants.n_motors)]
            await self.mc.move_all(positions, speeds)



class Sequences(Enum):
    random = RandomSequence
    wave = 1