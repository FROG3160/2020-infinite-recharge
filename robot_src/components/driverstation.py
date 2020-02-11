import wpilib
from .common import remap
from networktables import NetworkTables


class FROGStick(wpilib.Joystick):

    DEADBAND = 0.15
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        self.setThrottleChannel(3)
        self.setTwistChannel(2)
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.init_NT()

    def init_NT(self):
        NetworkTables.initialize()
        self.joystick_values = NetworkTables.getTable("joystick_values")

    def update_NT(self, control, value):
        self.joystick_values.putNumber(control, value)

    def getSpeed(self):
        # Dampens the -1 to 1 values of the joystick to provide a smoothed acceleration
        speed = self.getY()
        speed = -1 * (speed ** 3 / self.SPEED_DIVISOR
                      if abs(speed) > self.DEADBAND
                      else 0
                      )
        self.update_NT('speed', speed)
        return speed

    def getThrottle(self):
        val = super().getThrottle()
        self.update_NT('throttle', val)
        return val

    def getRotation(self):
        return (
            self.getTwist() / self.ROTATION_DIVISOR
            if abs(self.getTwist()) > self.DEADBAND
            else 0
        )

    def getRangedCubedRotation(self):
        return remap(
            self.getTwist() ** 3,
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,

        )

    def getRangeRotation(self):
        return remap(
            self.getTwist(),
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def getDebouncedButton(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""

        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True

        val = False
        self.update_NT('button_{}'.format(num), val)
        return val

