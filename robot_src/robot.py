#!/usr/bin/env python3

import magicbot
from magicbot import feedback
import wpilib
from wpilib import SmartDashboard as SD
from ctre import WPI_TalonFX, WPI_TalonSRX, CANifier, WPI_VictorSPX
from components.drivetrain import FROGDrive
from components.driverstation import FROGXboxGunner, FROGStickBase
from components.shooter import (
    FROGShooter,
    Azimuth,
    Elevation,
    Flywheel,
    Loader,
    Turret,
    Intake,
    Feed,
)

# from components.lift import Lift
from components.sensors import FROGdar, FROGGyro, LimitSwitch
from components.led import FROGLED
from components.vision import FROGVision
from components.common import Buffer

LEFTHAND = wpilib.XboxController.Hand.kLeftHand
RIGHTHAND = wpilib.XboxController.Hand.kRightHand

# controller modes
NORMAL = 1
MANUAL = 0


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """

    lidar: FROGdar
    gyro: FROGGyro
    chassis: FROGDrive  # controls drivetrain
    shooter: FROGShooter  # controls flywheel and feed
    loader: Loader  # controls intake
    vision: FROGVision
    turret: Turret

    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel
    feed: Feed
    intake: Intake

    def createObjects(self):
        """Create motors and inputs"""
        # add smartdashboard values
        SD.putNumber('driver_DEADBAND', 0
        15)
        SD.putNumber('driver_SPEED_DIVISOR', 1)
        SD.putNumber('driver_ROTATION_DIVISOR', 1.6)
        SD.putNumber('driver_DEBOUNCE_PERIOD', 0.5)
        # chassis components
        self.leftMaster = WPI_TalonFX(11)
        self.rightMaster = WPI_TalonFX(12)
        self.leftSlave = WPI_TalonFX(13)
        self.rightSlave = WPI_TalonFX(14)

        self.intake_motor = WPI_VictorSPX(21)
        # self.conveyor_motor = WPI_VictorSPX(22)
        self.feed_motor = WPI_TalonFX(23)

        self.azimuth_motor = WPI_TalonSRX(31)
        self.elevation_motor = WPI_TalonSRX(32)
        self.flywheel_motor = WPI_TalonFX(33)

        # self.controlwheel_motor = WPI_TalonSRX(41)

        # self.liftLeft = WPI_TalonSRX(51)
        # self.liftRight = WPI_TalonSRX(52)

        self.front_limit = LimitSwitch(0)
        self.rear_limit = LimitSwitch(1)

        # controls
        # self.drive_stick = FROGXboxDriver(0)
        self.drive_stick = FROGStickBase(0)
        self.gunner_stick = FROGXboxGunner(1)

        # used for LIDAR
        self.pwm_sensor = CANifier(39)

        # other objects

        self.led = FROGLED(0, 144)

        self.driverMode = NORMAL
        self.gunnerMode = NORMAL
        self.loopcount = 0

    def getDriverInputs(self):
        """Driver Controls:
        Normal:
            Left Bumper: Intake
            Right Bumber: Drive to Target
            Right Trigger: Forward
            Left Trigger: Reverse
            Left Stick: Rotate
            Left Stick push: reset gyro
        """
        # allow the driver to zero the gyro
        if self.drive_stick.getButtonDebounced(4):
            self.gyro.resetGyro()

        if self.drive_stick.getButtonDebounced(3):
            self.loader.manual_enable()
        else:
            self.loader.manual_disable()

        # read the values off of SmartDashboard

        if self.drive_stick.getButton(1):
            pc_x = self.vision.getPowerCellErrorX()
            pc_y = self.vision.getPowerCellErrorY()
            if pc_x and pc_y:
                if pc_y < 60:
                    self.loader.auto_enable()
                else:
                    self.loader.auto_disable()
                self.chassis.driveToTarget(pc_x, pc_y)
        else:
            self.loader.auto_disable()
            self.chassis.set_velocity(
                self.drive_stick.getSpeed(), self.drive_stick.getRotation()
            )

    @feedback(key="GunnerMode")
    def getGunnerMode(self):
        return self.gunnerMode == NORMAL

    def getGunnerInputs(self):
        # normal mode:
        #   right trigger = fire
        #   start button = switch to manual mode
        #   right stick push = reset Elevation encoder
        #
        # manual mode:
        #   back button = switch to normal mode
        #   d-pad up = increase flywheel velocity
        #   d-pad down = decrease flywheel velocity
        #   right bumper = run intake manually
        #   right trigger = fire
        #   left stick push = reset azimuth encoder

        if self.gunnerMode == NORMAL:
            # the small button on the right
            if self.gunner_stick.getStartButtonPressed():
                self.gunnerMode = MANUAL
                self.turret.disable()
                self.elevation.enable()
                self.azimuth.enable()

            if self.gunner_stick.getTriggerAxis(LEFTHAND) > 0.25:
                self.shooter.enable()
                if self.gunner_stick.getTriggerAxis(RIGHTHAND) > 0.25:
                    self.shooter.fire()
                else:
                    self.shooter.ceaseFire()
            else:
                self.shooter.disable()

            if self.gunner_stick.getBumper(RIGHTHAND) == 1:
                pass

            if self.gunner_stick.getStickButtonPressed(RIGHTHAND):
                self.elevation.resetEncoder()

        elif self.gunnerMode == MANUAL:
            # the small button on the left
            if self.gunner_stick.getBackButtonPressed():
                self.gunnerMode = NORMAL
                self.azimuth.disable()
                self.elevation.disable()
                self.turret.enable()

            if self.gunner_stick.get_debounced_POV() == 0:
                self.flywheel.incrementSpeed()

            elif self.gunner_stick.get_debounced_POV() == 180:
                self.flywheel.decrementSpeed()

            # runs the intake/loader manually
            if self.gunner_stick.getBumper(RIGHTHAND):
                self.loader.manual_enable()
            else:
                self.loader.manual_disable()

            if self.gunner_stick.getTriggerAxis(RIGHTHAND) > 0.25:
                self.shooter.enable()
            else:
                self.shooter.disable()

            # if self.gunner_stick.getAButtonPressed():
            #    self.flywheel.toggleVelocityMode()
            # if self.gunner_stick.getXButtonPressed():
            #    self.flywheel.enable()

            # if self.gunner_stick.getBButtonPressed():
            #    self.flywheel.disable()

            if self.gunner_stick.getStickButtonPressed(LEFTHAND):
                self.azimuth.resetEncoder()

            self.azimuth.setSpeed(self.gunner_stick.get_rotation())

            self.elevation.setSpeed(self.gunner_stick.get_elevation())

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.reset_encoders()
        self.chassis.init_velocity_mode()
        self.lidar.enable()
        self.turret.set_automatic()



    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        self.getDriverInputs()
        self.getGunnerInputs()

        self.led.setFROGGreen()

    def testInit(self):

        self.chassis.reset_encoders()
        self.ledFunc = Buffer(3)
        self.ledFunc.appendList(
            [self.led.setRed,
             self.led.setBlue,
             self.led.setGreen]
        )
        self.dioTest = LimitSwitch(0)

    def testPeriodic(self):

        if self.dioTest.isOpen():
            self.ledFunc[0]()
            self.ledFunc.rotate()

        # self.loopcount += 1
        # if (self.loopcount % 300) < 100:
        # self.led.setRed()
        # elif (self.loopcount % 300) < 200:
        # self.led.setGreen()
        # else:
        # self.led.setBlue()

        # self.led.setRainbow()
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
