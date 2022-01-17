from wpilib import AddressableLED


class PID:
    """Class that holds contants for PID controls"""

    # TODO: Add methods to apply to motor?

    def __init__(
        self, slot: int = 0, p: float = 0, i: float = 0, d: float = 0, f: float = 0
    ):
        self.slot = slot
        self.p = p
        self.i = i
        self.d = d
        self.f = f


def remap(val, OldMin, OldMax, NewMin, NewMax):
    '''take a value in the old range and return a value in the new range'''
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


def limit(val, min_val, max_val):
    '''keep a value within a certain range'''
    return max(min(max_val, val), min_val)


class LED(AddressableLED):
    def __init__(self, portNum, length):
        super().__init__(portNum)
        self.setLength(length)
        self.setData([self.LEDData(255, 0, 255)] * length)
        self.start()


if __name__ == '__main__':
    print(remap(0, 1, 0, 9000, 0.5) == 4500)
    print(remap(0, 1, 0, 9000, -0.3))
    print(limit(2, -1, 1))
    print(limit(-123, 0, 900))
    print(limit(45, 0, 100))
    print(limit(-32, -100, 0))
