from collections import deque
from magicbot import feedback, tunable
from ctre import CANifier
from navx import AHRS
from .common import Buffer
from wpilib import DigitalInput

BUFFERLEN = 25  # half a second?
SENSORUNITS_IN_INCHES = 0.0394


class LimitSwitch(DigitalInput):
    def __init__(self, dio):
        super().__init__(dio)

    def isOpen(self):
        # DigitalInput.get() returns True if signal line is open
        return self.get()

    def isClosed(self):
        return not self.get()


class FROGdar:
    pwm_sensor: CANifier

    def __init__(self):
        self.enabled = False
        self.targetRange = None
        self.rangeBuffer = Buffer(BUFFERLEN)
        # self.rangeBuffer = deque(maxlen=BUFFERLEN)

    def disable(self):
        self.enabled = False

    def enable(self):
        # clear range data to get fresh information
        self.targetRange = None
        self.rangeBuffer.clear()
        self.enabled = True

    def isValidData(self):
        return (
            self.rangeBuffer.lengthFiltered() >= BUFFERLEN
            and not self.targetRange is None
        )

    @feedback(key='LIDAR')
    def getSensorData(self):

        errorcode, (val1, val2) = self.pwm_sensor.getPWMInput(
            CANifier.PWMChannel.PWMChannel0
        )
        return val1

    @feedback(key="distance")
    def getDistance(self):
        if self.isValidData():
            return self.targetRange * SENSORUNITS_IN_INCHES

    def execute(self):
        if self.enabled:
            # stream data into our counter
            self.rangeBuffer.append(self.getSensorData())
            if self.rangeBuffer.lengthFiltered() > 0:
                self.targetRange = self.rangeBuffer.average()


class FROGGyro:
    def __init__(self):
        self.gyro = AHRS.create_spi()
        self.gyro.reset()

    @feedback(key='Heading')
    def getHeading(self):
        # returns gyro heading +180 to -180 degrees
        return self.gyro.getYaw()

    def resetGyro(self):
        self.gyro.reset()
