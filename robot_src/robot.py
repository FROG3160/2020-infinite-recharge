#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    a single joystick
"""
import magicbot
import wpilib
from ctre import WPI_TalonFX, WPI_TalonSRX
from components.drivetrain import FROGDrive
from components.driverstation import FROGStick, FROGXboxDriver, FROGXboxGunner
from components.turret import FROGTurret, Shooter


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components here.
    """

    chassis: FROGDrive
    turret: FROGTurret

    shooter: Shooter

    def createObjects(self):
        """Create motors and inputs"""
        # chassis components
        self.leftMaster = WPI_TalonFX(11)
        self.rightMaster = WPI_TalonFX(12)
        self.leftSlave = WPI_TalonFX(13)
        self.rightSlave = WPI_TalonFX(14)

        self.intake = WPI_TalonSRX(21)
        self.lowerConveyor = WPI_TalonFX(22)
        self.upperConveyor = WPI_TalonFX(23)

        self.azimuth = WPI_TalonSRX(31)
        self.elevation = WPI_TalonSRX(32)
        self.flywheel = WPI_TalonFX(33)

        self.controllerWheel = WPI_TalonSRX(41)

        # controls
        self.drive_stick = FROGXboxDriver(0)
        self.gunner_stick = FROGXboxGunner(1)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.reset_encoders()
        self.chassis.init_velocity_mode()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        if self.drive_stick.get_debounced_button(11) == True:
            self.chassis.toggle_control_mode()

        if self.drive_stick.getPOV() == 0:
            self.chassis.set_position(36)  # tell it to roll forward 36 inches
        if self.drive_stick.getPOV() == 180:
            self.chassis.set_position(-36)  # tell it to roll backward 36 inches
        # feed joystick to the drivetrain
        self.chassis.set_velocity(
            self.drive_stick.get_speed(), self.drive_stick.get_rotation()
        )
        self.shooter.setAzimuthPercent(self.gunner_stick.get_rotation())
        self.shooter.setElevationPercent(self.gunner_stick.get_elevation())
        if self.gunner_stick.get_debounced_POV() == 0:
            self.shooter.incrementFlywheelPercent()
        elif self.gunner_stick.get_debounced_POV() == 180:
            self.shooter.decrementFlywheelPercent()
        if self.gunner_stick.getBButtonPressed():
            self.shooter.stopFlywheelPercent()

        self.gunner_stick.setRumble(arg1, arg2)

    def testInit(self):

        self.chassis.reset_encoders()
        self.chassis.init_position_mode()

    def testPeriodic(self):

        if self.drive_stick.get_button(12):
            self.turret.shooter.setFlywheelPercent(self.drive_stick.get_throttle())


if __name__ == "__main__":
    wpilib.run(FROGbot)
