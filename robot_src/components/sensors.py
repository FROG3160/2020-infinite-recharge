from magicbot import feedback
from ctre import CANifier
from navx import AHRS
from .common import Buffer
from wpilib import DigitalInput

# if we are getting a reading every 20ms loop, then a
# buffer of 50 should be a second's worth of data.
# increase this if the values jump around too much,
# decrease if it's not reacting to change fast enough.
BUFFERLEN = 50

# the LIDAR readings are in microseconds, and 10 microseconds = 1 cm.
# therefore .394 in/cm * 1 cm/10 microseconds =
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

    def disable(self):
        self.enabled = False

    def enable(self):
        # clear range data to get fresh information
        self.rangeBuffer.clear()
        self.enabled = True

    def isValidData(self):
        return (
            self.rangeBuffer.lengthFiltered() >= BUFFERLEN
            and not self.targetRange is None
        )

    @feedback(key='raw')
    def getSensorData(self):
        errorcode, (val1, val2) = self.pwm_sensor.getPWMInput(
            CANifier.PWMChannel.PWMChannel0
        )
        return val1

    @feedback(key='buffered')
    def getBufferedSensorData(self):
        return self.targetRange

    @feedback(key="distance")
    def getDistance(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_INCHES

    def execute(self):
        if self.enabled:
            # stream data into our counter
            self.rangeBuffer.append(self.getSensorData())
            if self.rangeBuffer.lengthFiltered() > 0:
                self.targetRange = self.rangeBuffer.average()
            else:
                self.targetRange = None
        else:
            self.rangeBuffer.clear() 
            self.targetRange = None


class FROGGyro:
    def __init__(self):
        self.gyro = AHRS.create_spi()
        self.gyro.reset()

    @feedback(key='heading')
    def getHeading(self):
        # returns gyro heading +180 to -180 degrees
        return self.gyro.getYaw()

    def resetGyro(self):
        # sets yaw reading to 0
        self.gyro.reset()

    def execute(self):
        pass
