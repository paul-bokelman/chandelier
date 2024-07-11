from sequences.sequence import Sequence

# sine wave with steps for each candle

class WaveSequence(Sequence):
    def __init__(self, data):
        super().__init__(data)

    def start(self):
        pass