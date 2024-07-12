from enum import Enum

class Sequence:
    def __init__(self, data):
        self.data = data

    def start(self):
        pass

    def stop(self):
        pass
    
    def reset(self):
        pass

class RandomSequence(Sequence):
    def __init__(self, data):
        super().__init__(data)

    def start(self):
        pass

    def stop(self):
        pass
    
    def reset(self):
        pass


class Sequences(Enum):
    random = RandomSequence
    wave = 1