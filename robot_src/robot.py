#!/usr/bin/env python3

import magicbot
import wpilib
from ctre import WPI_TalonFX, WPI_TalonSRX
from components.drivetrain import FROGDrive
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
from components.common import LED


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

    def createObjects(self):
        """Create motors and inputs"""
        # chassis components
        self.leftMaster = WPI_TalonFX(11)
        self.rightMaster = WPI_TalonFX(12)
        self.leftSlave = WPI_TalonFX(13)
        self.rightSlave = WPI_TalonFX(14)

        self.intake_motor = WPI_TalonSRX(21)
        self.conveyor_motor = WPI_TalonSRX(22)
        self.loader_motor = WPI_TalonSRX(23)

        self.azimuth_motor = WPI_TalonSRX(31)
        self.elevation_motor = WPI_TalonSRX(32)
        self.flywheel_motor = WPI_TalonFX(33)

        self.controlwheel_motor = WPI_TalonSRX(41)

        self.liftLeft = WPI_TalonFX(51)
        self.liftRight = WPI_TalonFX(52)

        # controls
        self.drive_stick = FROGXboxDriver(0)
        self.gunner_stick = FROGXboxGunner(1)

        self.led = LED(0, 29)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.reset_encoders()
        self.chassis.init_velocity_mode()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        if self.drive_stick.get_debounced_button(11) == True:
            self.chassis.toggle_control_mode()
        if self.drive_stick.getStickButtonPressed(wpilib.XboxController.Hand.kLeftHand):
            self.chassis.resetGyro()

        if self.drive_stick.getPOV() == 0:
            self.chassis.set_position(36)  # tell it to roll forward 36 inches
        if self.drive_stick.getPOV() == 180:
            self.chassis.set_position(-36)  # tell it to roll backward 36 inches
        # feed joystick to the drivetrain
        self.chassis.set_velocity(
            self.drive_stick.get_speed(), self.drive_stick.get_rotation()
        )

        self.azimuth.setSpeed(self.gunner_stick.get_rotation())
        self.azimuth.enable()
        self.elevation.set_speed(self.gunner_stick.get_elevation())
        self.elevation.enable()
        if self.gunner_stick.get_debounced_POV() == 0:
            self.flywheel.incrementSpeed()
            self.flywheel.enable()
        elif self.gunner_stick.get_debounced_POV() == 180:
            self.flywheel.decrementSpeed()
            self.flywheel.enable()
        if self.gunner_stick.getBButtonPressed():
            self.flywheel.disable()

    def testInit(self):

        self.chassis.reset_encoders()
        self.chassis.init_position_mode()

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
