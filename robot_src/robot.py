#!/usr/bin/env python3

import magicbot
import wpilib
from ctre import WPI_TalonFX, WPI_TalonSRX, CANifier, WPI_VictorSPX
from components.drivetrain import FROGDrive, POSITION_MODE, VELOCITY_MODE
from components.driverstation import FROGXboxDriver, FROGXboxGunner
from components.shooter import (
    FROGShooter,
    Azimuth,
    Elevation,
    Flywheel,
    Loader,
    # Conveyor,
    Intake,
)

# from components.lift import Lift
from components.sensors import FROGdar, FROGGyro, LimitSwitch
from components.led import FROGLED
from components.vision import FROGVision
from components.common import Buffer

LEFTHAND = wpilib.XboxController.Hand.kLeftHand
RIGHTHAND = wpilib.XboxController.Hand.kRightHand

# controller modes
NORMAL = 0
MANUAL = 1


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components here.
    """

    lidar: FROGdar
    gyro: FROGGyro

    chassis: FROGDrive
    # shooter: FROGShooter

    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel
    loader: Loader
    # conveyor: Conveyor
    intake: Intake
    # lift: Lift
    vision: FROGVision

    def createObjects(self):
        """Create motors and inputs"""
        # chassis components
        self.leftMaster = WPI_TalonFX(11)
        self.rightMaster = WPI_TalonFX(12)
        self.leftSlave = WPI_TalonFX(13)
        self.rightSlave = WPI_TalonFX(14)

        self.intake_motor = WPI_VictorSPX(21)
        # self.conveyor_motor = WPI_VictorSPX(22)
        self.loader_motor = WPI_TalonFX(23)

        self.azimuth_motor = WPI_TalonSRX(31)
        self.elevation_motor = WPI_TalonSRX(32)
        self.flywheel_motor = WPI_TalonFX(33)

        # self.controlwheel_motor = WPI_TalonSRX(41)

        # self.liftLeft = WPI_TalonSRX(51)
        # self.liftRight = WPI_TalonSRX(52)

        self.front_limit = LimitSwitch(0)
        self.rear_limit = LimitSwitch(1)

        # controls
        self.drive_stick = FROGXboxDriver(0)
        self.gunner_stick = FROGXboxGunner(1)

        # used for LIDAR
        self.pwm_sensor = CANifier(39)

        # other objects

        self.led = FROGLED(0, 144)

        self.driverMode = NORMAL
        self.gunnerMode = MANUAL
        self.loopcount = 0

    def getDriverInputs(self):
        """Driver Controls:
           Normal:
               Left Bumper: Intake
               Right Bumber: Drive to Target
               Right Trigger: Forward
               Left Trigger: Reverse
               Left Stick: Rotate
               """
        # allow the driver to zero the gyro
        if self.drive_stick.getStickButtonPressed(LEFTHAND):
            self.gyro.resetGyro()

        # Right Bumper changes between position control with d-pad (POV)
        # and driving the robot with the joystick/trigger
        # if self.drive_stick.getBumperPressed(RIGHTHAND):
        # self.chassis.toggle_control_mode()

        if self.drive_stick.getBumper(LEFTHAND):
            self.intake.enable()
            self.conveyor.enable()
        else:
            self.intake.disable()
            # self.conveyor.disable()

        if self.chassis.control_mode == POSITION_MODE:
            pov = self.drive_stick.get_debounced_POV()
            if pov == 0:
                self.chassis.set_position(36)  # tell it to roll forward 36 inches
            elif pov == 180:
                self.chassis.set_position(-36)  # tell it to roll backward 36 inches
            elif pov == 90:
                self.chassis.set_rotate(45)
            elif pov == 270:
                self.chassis.set_rotate(-45)
        else:
            if self.drive_stick.getBumper(RIGHTHAND):
                self.chassis.driveToTarget()
            else:
                self.chassis.set_velocity(
                    self.drive_stick.get_speed(), self.drive_stick.get_rotation()
                )
            # self.lift.set_speed(self.drive_stick.getY(RIGHTHAND))
            # self.lift.enable()

    def getGunnerInputs(self):

        if self.gunnerMode == NORMAL:
            # run the shooter state machine

            # the small button on the right
            if self.gunner_stick.getStartButton():
                self.gunnerMode = MANUAL
                self.intake.disable()
                self.conveyor.disable()
                self.loader.disable()
                self.flywheel.disable()
                self.azimuth.setManual()
                self.elevation.setManual()

            if self.gunner_stick.getTriggerAxis(RIGHTHAND) == 1:
                pass

            if self.gunner_stick.getBumper(RIGHTHAND) == 1:
                pass

        elif self.gunnerMode == MANUAL:
            # the small button on the left
            if self.gunner_stick.getBackButton():
                self.gunnerMode = NORMAL
                self.azimuth.setAutomatic()
                self.elevation.setAutomatic()

            if self.gunner_stick.get_debounced_POV() == 0:
                self.flywheel.incrementSpeed()

            elif self.gunner_stick.get_debounced_POV() == 180:
                self.flywheel.decrementSpeed()

            # runs the belts using the bumpers
            if self.gunner_stick.getBumper(RIGHTHAND):
                self.loader.override = True
                self.loader.enable()
                # self.conveyor.enable()

            # if self.gunner_stick.getBumper(LEFTHAND):
            #    self.intake.enable()

            if self.gunner_stick.getTriggerAxis(RIGHTHAND) > 0.25:
                # self.loader.override = True
                # self.loader.enable()
                # self.intake.enable()
                # self.conveyor.enable()
                self.flywheel.enable()
            else:
                self.loader.override = False
                self.loader.disable()
                self.intake.disable()
                # self.conveyor.disable()
                self.flywheel.disable()

            if self.gunner_stick.getAButtonPressed():
                self.flywheel.toggleVelocityMode()
            if self.gunner_stick.getXButtonPressed():
                self.flywheel.enable()

            if self.gunner_stick.getBButtonPressed():
                self.flywheel.disable()

            if self.gunner_stick.getStickButtonPressed(LEFTHAND):
                self.azimuth.resetEncoder()

            # self.azimuth.setSpeed(self.gunner_stick.get_rotation())
            # self.azimuth.enable()
            # self.elevation.set_speed(self.gunner_stick.get_elevation())
            # self.elevation.enable()

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.reset_encoders()
        self.chassis.init_velocity_mode()
        self.lidar.enable()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        self.getDriverInputs()
        self.getGunnerInputs()

        self.led.setFROGGreen()

    def testInit(self):

        self.chassis.reset_encoders()
        self.chassis.init_position_mode()
        self.ledFunc = Buffer(3)
        self.ledFunc.appendList([self.led.setRed, self.led.setBlue, self.led.setGreen])
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
