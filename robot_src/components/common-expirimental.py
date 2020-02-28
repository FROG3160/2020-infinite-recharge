from wpilib import SendableBuilder, AddressableLED


class PID(SendableBuilder):
    """Class that holds contants for PID controls"""

    # TODO: Add methods to apply to motor?

    def __init__(
        self, slot: int = 0, p: float = 0, i: float = 0, d: float = 0, f: float = 0
    ):
        self.setSmartDashboardType('RobotPreferences')
        self.addDoubleArrayProperty('P', self.getP, self.setP)
        self.addDoubleArrayProperty('I', self.getI, self.setI)
        self.addDoubleArrayProperty('D', self.getD, self.setD)
        self.addDoubleArrayProperty('F', self.getF, self.setF)
        self.slot = slot
        self.p = p
        self.i = i
        self.d = d
        self.f = f

    def setP(self, value):
        self.p = value

    def setI(self, value):
        self.i = value

    def setD(self, value):
        self.d = value

    def setF(self, value):
        self.f = value

    def getP(self):
        return self.p

    def getI(self, value):
        return self.i

    def getD(self, value):
        return self.d

    def getF(self, value):
        return self.f


def remap(val, OldMin, OldMax, NewMin, NewMax):
    '''take a value in the old range and return a value in the new range'''
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


def limit(val, min_val, max_val):
    '''keep a value within a certain range'''
    return max(min(max_val, val), min_val)


if __name__ == '__main__':
    print(remap(0, 1, 0, 9000, 0.5) == 4500)
    print(remap(0, 1, 0, 9000, -0.3))
    print(limit(2, -1, 1))
    print(limit(-123, 0, 900))
    print(limit(45, 0, 100))
    print(limit(-32, -100, 0))
