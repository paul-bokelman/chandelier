from sequences.sequence import Sequence

class RandomSequence(Sequence):
    def __init__(self, data):
        super().__init__(data)
        self.data = data
        self.index = 0

    def test(self):
        pass