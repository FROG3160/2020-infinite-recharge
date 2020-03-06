#!/usr/bin/env python3

import magicbot
import wpilib
from ctre import WPI_TalonFX, WPI_TalonSRX, WPI_VictorSPX
from components.drivetrain import FROGDrive, POSITION_MODE, VELOCITY_MODE
from components.driverstation import FROGStick, FROGXboxDriver, FROGXboxGunner
from components.shooter import (
    FROGShooter,
    Azimuth,
    Elevation,
    Flywheel,
    Loader,
    Conveyor,
    Intake,
)
from components.lift import Lift
from components.common import LED

LEFTHAND = wpilib.XboxController.Hand.kLeftHand
RIGHTHAND = wpilib.XboxController.Hand.kRightHand


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components here.
    """

    chassis: FROGDrive
    shooter: FROGShooter

    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel
    loader: Loader
    conveyor: Conveyor
    intake: Intake
    lift: Lift

    def createObjects(self):
        """Create motors and inputs"""
        # chassis components
        self.leftMaster = WPI_TalonFX(11)
        self.rightMaster = WPI_TalonFX(12)
        self.leftSlave = WPI_TalonFX(13)
        self.rightSlave = WPI_TalonFX(14)

        self.intake_motor = WPI_VictorSPX(21)
        self.conveyor_motor = WPI_VictorSPX(22)
        self.loader_motor = WPI_TalonSRX(23)

        self.azimuth_motor = WPI_TalonSRX(31)
        self.elevation_motor = WPI_TalonSRX(32)
        self.flywheel_motor = WPI_TalonFX(33)

        # self.controlwheel_motor = WPI_TalonSRX(41)

        self.liftLeft = WPI_TalonSRX(51)
        self.liftRight = WPI_TalonSRX(52)

        # controls
        self.drive_stick = FROGXboxDriver(0)
        self.gunner_stick = FROGXboxGunner(1)

        self.led = LED(0, 29)

    def getDriverInputs(self):
        # allow the driver to zero the gyro

        if self.drive_stick.getStickButtonPressed(LEFTHAND):
            self.chassis.resetGyro()

        # Right Bumper changes between position control with d-pad (POV)
        # and driving the robot with the joystick/trigger
        if self.drive_stick.getBumperPressed(RIGHTHAND):
            self.chassis.toggle_control_mode()

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
            self.chassis.set_velocity(
                self.drive_stick.get_speed(), self.drive_stick.get_rotation()
            )

    def getGunnerInputs(self):
        if self.gunner_stick.getTriggerAxis(RIGHTHAND) == 1:
            self.shooter.fire()

        if self.gunner_stick.get_debounced_POV() == 0:
            self.flywheel.incrementSpeed()
            self.flywheel.enable()
        elif self.gunner_stick.get_debounced_POV() == 180:
            self.flywheel.decrementSpeed()
            self.flywheel.enable()
        if self.gunner_stick.getBButtonPressed():
            self.flywheel.disable()
            self.azimuth.disable()

        # runs the belts using the bumpers
        if self.gunner_stick.getBumper(RIGHTHAND):
            self.conveyor.enable()
        else:
            self.conveyor.disable()
        if self.gunner_stick.getBumper(LEFTHAND):
            self.intake.enable()
        else:
            self.intake.disable()

        self.azimuth.setSpeed(self.gunner_stick.get_rotation())
        self.azimuth.enable()
        self.elevation.set_speed(self.gunner_stick.get_elevation())
        self.elevation.enable()

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.reset_encoders()
        self.chassis.init_velocity_mode()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        self.getDriverInputs()
        self.getGunnerInputs()

    def testInit(self):

        self.chassis.reset_encoders()
        self.chassis.init_position_mode()

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
