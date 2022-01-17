import wpilib
from wpilib import Joystick, XboxController
from wpilib.interfaces import GenericHID
from .common import remap
from networktables import NetworkTables
from magicbot import feedback


LEFTHAND = XboxController.Hand.kLeftHand
RIGHTHAND = XboxController.Hand.kRightHand

LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble
RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble


class FROGStickBase(Joystick):

    DEADBAND = 0.15
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, port, name='FROGStick'):
        super().__init__(port)
        self.name = name
        self.port = port
        # set channel numbers for the joystix axis
        # this can be determined in the driver station
        self.setXChannel(0)
        self.setYChannel(1)
        self.setThrottleChannel(2)
        self.setTwistChannel(3)

        self.button_latest = {}
        self.timer = wpilib.Timer
        # TODO: see if the network tables is needed
        # I think all objects are already place on NT
        # self.nt = NetworkTables.getTable("{}_values".format(self.name))

    # def update_NT(self, control, value):
    #     self.nt.putNumber(control, value)

    @feedback
    def getSpeed(self):
        # Dampens the -1 to 1 values of the joystick
        # to provide a smoothed acceleration
        speed = self.getY()
        speed = -1 * (
            speed ** 3 / self.SPEED_DIVISOR
            if abs(speed) > self.DEADBAND
            else 0
        )
    #    self.update_NT('speed', speed)
        return speed

    @feedback
    def get_rotation(self):
        return (
            self.getTwist() / self.ROTATION_DIVISOR
            if abs(self.getTwist()) > self.DEADBAND
            else 0
        )

    def getButton(self, num):
        val = self.getRawButton(num)
        # self.update_NT('button_{}'.format(num), val)
        return val

    def getButtonDebounced(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        # self.update_NT('button_{}'.format(num), val)
        return val


class driveFROGStick(FROGStickBase):
    pass


class FROGXboxSimple(wpilib.XboxController):
    DEADBAND = 0.1
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        # self.setThrottleChannel(3)
        # self.setTwistChannel()
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.nt = NetworkTables.getTable("FROGXboxSimple_values")

    @feedback
    def get_speed(self):
        # Dampens the -1 to 1 values of the joystick
        # to provide a smoothed acceleration
        speed = self.getY(LEFTHAND)
        speed = -1 * (
            speed ** 3 / self.SPEED_DIVISOR
            if abs(speed) > self.DEADBAND
            else 0
        )
        return speed

    @feedback
    def get_rotation(self):
        return (
            self.getX(LEFTHAND) / self.ROTATION_DIVISOR
            if abs(self.getX(LEFTHAND)) > self.DEADBAND
            else 0
        )

    def get_debounced_button(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        self.update_nt('button_{}'.format(num), val)
        return val

    def update_nt(self, key, value):
        """update network tables with drive telemetry"""
        self.nt.putNumber(key, value)


class FROGXboxDriver(wpilib.XboxController):
    DEADBAND = 0.1
    SPEED_DIVISOR = 1.2
    ROTATION_DIVISOR = 1.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        # self.setThrottleChannel(3)
        # self.setTwistChannel()
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.nt = NetworkTables.getTable("FROGXboxDriver_values")

    @feedback(key='Speed')
    def get_speed(self):
        # Dampens the -1 to 1 values of the joystick
        # to provide a smoothed acceleration
        forward = self.getTriggerAxis(RIGHTHAND)
        backward = self.getTriggerAxis(LEFTHAND)
        speed = forward - backward
        speed = (
            speed ** 3 / self.SPEED_DIVISOR
            if abs(speed) > self.DEADBAND
            else 0
        )
        return speed

    @feedback(key='Rotation')
    def get_rotation(self):
        return (
            self.getX(LEFTHAND) / self.ROTATION_DIVISOR
            if abs(self.getX(LEFTHAND)) > self.DEADBAND
            else 0
        )

    def get_debounced_button(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        self.update_nt('button_{}'.format(num), val)
        return val

    def get_debounced_POV(self):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        if pov > -1:
            if (now - self.button_latest.get('POV', 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest['POV'] = now
                val = pov
                self.setRumble(GenericHID.RumbleType.kRightRumble, 1)
            else:
                self.setRumble(GenericHID.RumbleType.kRightRumble, 0)
        self.update_nt('button_pov', val)
        return val

    def update_nt(self, key, value):
        """update network tables with drive telemetry"""
        self.nt.putNumber(key, value)


class FROGXboxGunner(wpilib.XboxController):
    DEADBAND = 0.1
    ELEVATION_DIVISOR = 1
    ROTATION_DIVISOR = 1
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.nt = NetworkTables.getTable(
            "components/driverstation/gunner_stick"
        )

    @feedback(key='Elevation')
    def get_elevation(self):
        return (
            self.getY(RIGHTHAND) / self.ELEVATION_DIVISOR
            if abs(self.getX(RIGHTHAND)) > self.DEADBAND
            else 0
        )

    @feedback(key='Rotation')
    def get_rotation(self):
        return (
            self.getX(LEFTHAND) / self.ROTATION_DIVISOR
            if abs(self.getX(LEFTHAND)) > self.DEADBAND
            else 0
        )

    def get_debounced_button(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        self.update_nt('button_{}'.format(num), val)
        return val

    def get_debounced_POV(self):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        # if button is pressed and we are outside the debounce period,
        # set value
        if pov > -1:
            if (now - self.button_latest.get('POV', 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest['POV'] = now
                val = pov
        # if we are inside the debounce period, turn on rumble.
        if (now - self.button_latest.get('POV', 0)) < self.DEBOUNCE_PERIOD:
            self.setRumble(RIGHT_RUMBLE, 1)
        else:
            self.setRumble(RIGHT_RUMBLE, 0)
        self.update_nt('button_pov', val)
        return val

    def update_nt(self, key, value):
        """update network tables with drive telemetry"""
        self.nt.putNumber(key, value)
