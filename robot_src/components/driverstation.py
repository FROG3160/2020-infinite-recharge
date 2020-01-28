import wpilib


def changeRange(OldMin, OldMax, NewMin, NewMax, Value):
    return (((Value - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


class FROGStick(wpilib.Joystick):

    DEADBAND = 0.1
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        self.setThrottleChannel(3)
        self.setTwistChannel(2)

    def getSpeed(self):
        # Dampens the -1 to 1 values of the joystick to provide a smoothed acceleration
        return (
            -self.getY() ** 3 / self.SPEED_DIVISOR
            if abs(self.getY()) > self.DEADBAND
            else 0
        )

    def getRotation(self):
        return (
            self.getTwist() / self.ROTATION_DIVISOR
            if abs(self.getTwist()) > self.DEADBAND
            else 0
        )

    def getRangedCubedRotation(self):
        return changeRange(
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
            self.getTwist() ** 3,
        )

    def getRangeRotation(self):
        return changeRange(
            self.SPEED_DIVISOR, 1, self.ROTATION_MIN, self.ROTATION_MAX, self.getTwist()
        )